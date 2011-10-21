/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.hokuyo;

import com.google.common.base.Preconditions;

import android.util.Log;
import org.ros.exception.RosRuntimeException;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.Charset;
import java.util.Arrays;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class Scip20Device {

  private static final boolean DEBUG = false;
  private static final String TAG = "Scip20Device";

  private static final int STREAM_BUFFER_SIZE = 8192;

  private final BufferedReader reader;
  private final BufferedWriter writer;
  
  private double scanOffset;

  /**
   * It is not necessary to provide buffered streams. Buffering is handled
   * internally.
   * 
   * @param inputStream
   *          the {@link InputStream} for the ACM serial device
   * @param outputStream
   *          the {@link OutputStream} for the ACM serial device
   */
  public Scip20Device(InputStream inputStream, OutputStream outputStream) {
    // TODO(damonkohler): Wrapping the AcmDevice InputStream in an
    // BufferedInputStream avoids an error returned by the USB stack. Double
    // buffering like this should not be necessary if the USB error turns out to
    // be an Android bug. This was tested on Honeycomb MR2.
    reader =
        new BufferedReader(new InputStreamReader(new BufferedInputStream(inputStream,
            STREAM_BUFFER_SIZE), Charset.forName("US-ASCII")));
    writer =
        new BufferedWriter(new OutputStreamWriter(new BufferedOutputStream(outputStream,
            STREAM_BUFFER_SIZE), Charset.forName("US-ASCII")));
  }

  private void write(String command) {
    Preconditions.checkArgument(!command.endsWith("\n"));
    try {
      writer.write(command + "\n");
      writer.flush();
      if (DEBUG) {
        Log.d(TAG, "Wrote: " + command);
      }
    } catch (IOException e) {
      throw new RosRuntimeException(e);
    }
    String echo = read();
    Preconditions.checkState(echo.equals(command));
  }

  private void checkStatus() {
    String statusAndChecksum = read();
    String status = verifyChecksum(statusAndChecksum);
    if (status.equals("00") || status.equals("99")) {
      return;
    }
    throw new Scip20Exception(status);
  }

  private String read() {
    String line;
    try {
      line = reader.readLine();
    } catch (IOException e) {
      throw new RosRuntimeException(e);
    }
    if (DEBUG) {
      Log.d(TAG, "Read: " + line);
    }
    return line;
  }

  private String verifyChecksum(String buffer) {
    Preconditions.checkArgument(buffer.length() > 0);
    String data = buffer.substring(0, buffer.length() - 1);
    char checksum = buffer.charAt(buffer.length() - 1);
    int sum = 0;
    for (int i = 0; i < data.length(); i++) {
      sum += data.charAt(i);
    }
    if ((sum & 63) + 0x30 == checksum) {
      return data;
    }
    throw new InvalidChecksum();
  }

  public void reset() {
    try {
      write("RS");
      checkStatus();
      checkTerminator();
      // When we are in the middle of a scan, this might fail. 
      // Catch the exceptions and ignore.
    } catch(IllegalStateException e) {
    } catch(Scip20Exception e) {
    }

    // The second reset should actually work
    write("RS");
    checkStatus();
    checkTerminator();
    write("SCIP2.0");
    try {
      checkStatus();
    } catch (Scip20Exception e) {
      // This command is undefined for SCIP2.0 devices.
    }
    checkTerminator();
  }

  private void checkTerminator() {
    Preconditions.checkState(read().length() == 0);
  }

  private long readTimestamp() {
    String stampString = verifyChecksum(read());
    long stamp = 0;
    for(int i=0; i<4; i++)
      stamp |= stampString.charAt(i) << i;
    return stamp;
  }

  public void startScanning(final LaserScanListener listener) {
    new Thread() {
      @Override
      public void run() {
        String command = "MD0000076800000";
        write(command);
        checkStatus();
        checkTerminator();
        while (true) {
          Preconditions.checkState(read().equals(command));
          double scanStartTime = System.currentTimeMillis() / 1000.0;
          checkStatus();
          readTimestamp();
          StringBuilder data = new StringBuilder();
          while (true) {
            String line = read(); // Data and checksum or terminating LF
            if (line.length() == 0) {
              listener.onNewLaserScan(new LaserScan(scanStartTime + scanOffset, Decoder.decode(data.toString(), 3)));
              break;
            }
            data.append(verifyChecksum(line));
          }
        }
      }
    }.start();
  }

  private String readAndStripSemicolon() {
    String buffer = read();
    Preconditions.checkState(buffer.charAt(buffer.length() - 2) == ';');
    return buffer.substring(0, buffer.length() - 2) + buffer.charAt(buffer.length() - 1);
  }

  public Configuration queryConfiguration() {
    Configuration.Builder builder = new Configuration.Builder();
    write("PP");
    checkStatus();
    builder.parseModel(verifyChecksum(readAndStripSemicolon()));
    builder.parseMinimumMeasurement(verifyChecksum(readAndStripSemicolon()));
    builder.parseMaximumMeasurement(verifyChecksum(readAndStripSemicolon()));
    builder.parseTotalSteps(verifyChecksum(readAndStripSemicolon()));
    builder.parseFirstStep(verifyChecksum(readAndStripSemicolon()));
    builder.parseLastStep(verifyChecksum(readAndStripSemicolon()));
    builder.parseFrontStep(verifyChecksum(readAndStripSemicolon()));
    builder.parseStandardMotorSpeed(verifyChecksum(readAndStripSemicolon()));
    checkTerminator();
    return builder.build();
  }

  public void shutdown() {
    try {
      reader.close();
    } catch (IOException e) {
      // Ignore spurious shutdown errors.
      e.printStackTrace();
    }
    try {
      writer.close();
    } catch (IOException e) {
      // Ignore spurious shutdown errors.
      e.printStackTrace();
    }
  }
  
  // TODO(moesenle): assert that scanning is not running
  public void calibrateTime() {
    /* To calibrate time, we do the following (similar to what ROS' hokuyo_node does):
     *   1. get current hokuyo time and calculate offset to current time
     *   2. request a scan and calculate the scan offset to current time
     *   3. request hokuyo time again and calculate offset to current time
     *   4. offset = scan - (end + start)/2
     * We repeat this process 11 times and take the median offset.
     */
    long[] samples = new long[11];
    long start = hokuyoClockOffset();
    for(int i=0; i<samples.length; i++) {
      long scan = hokuyoScanOffset();
      long end = hokuyoClockOffset();
      samples[i] = scan - (end + start) / 2;
      start = end;
    }
    Arrays.sort(samples);
    scanOffset = samples[5] / 1000.0;
  }
  
  public double getScanOffset() {
    return scanOffset;
  }
  
  private long hokuyoClockOffset() {
    // Enter time adjust mode
    write("TM0");
    checkStatus();
    checkTerminator();

    // Read the current time stamp
    final long start = System.currentTimeMillis();
    write("TM1");
    checkStatus();
    final long offset = readTimestamp()
        - (start + System.currentTimeMillis()) / 2;
    checkTerminator();
    
    // Leave adjust mode
    write("TM2");
    checkStatus();
    checkTerminator();
    
    return offset;
  }
  
  private long hokuyoScanOffset() {
    // We request exactly one scan from the laser and use the difference 
    // between the laser's own time stamp and the system time at which we
    // received the scan
    write("MD0000076800001");
    checkStatus();
    checkTerminator();
    
    Preconditions.checkState(read().equals("MD0000076800000"));
    long scanStartTime = System.currentTimeMillis();
    checkStatus();
    long scanTimeStamp = readTimestamp();
    while(true) {
        String line = read(); // Data and checksum or terminating LF
        if (line.length() == 0) 
          break;
        verifyChecksum(line);
    }
    return scanTimeStamp - scanStartTime;
  }
}