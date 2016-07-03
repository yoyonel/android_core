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

package org.ros.android.view.camera;

import com.google.common.base.Preconditions;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera.Size;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.io.FileOutputStream;

/**
 * Publishes preview frames.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
class CompressedImagePublisher implements RawImageListener {

  private final ConnectedNode connectedNode;
  private final Publisher<sensor_msgs.CompressedImage> imagePublisher;
  private final Publisher<sensor_msgs.CameraInfo> cameraInfoPublisher;

  private byte[] rawImageBuffer;
  private Size rawImageSize;
  private YuvImage yuvImage;
  private Rect rect;
  private ChannelBufferOutputStream stream;

  public CompressedImagePublisher(ConnectedNode connectedNode) {
    this.connectedNode = connectedNode;
    NameResolver resolver = connectedNode.getResolver().newChild("camera");
    imagePublisher =
        connectedNode.newPublisher(resolver.resolve("image/compressed"),
            sensor_msgs.CompressedImage._TYPE);
    cameraInfoPublisher =
        connectedNode.newPublisher(resolver.resolve("camera_info"), sensor_msgs.CameraInfo._TYPE);
    stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
  }

  private int[] decodeGreyscale(byte[] nv21, int width, int height) {
    int pixelCount = width * height;
    int[] out = new int[pixelCount];
    for (int i = 0; i < pixelCount; ++i) {
      int luminance = nv21[i] & 0xFF;
      // out[i] = Color.argb(0xFF, luminance, luminance, luminance);
      out[i] = 0xff000000 | luminance <<16 | luminance <<8 | luminance;//No need to create Color object for each.
    }
    return out;
  }

  @Override
  public void onNewRawImage(byte[] data, Size size) {
    Preconditions.checkNotNull(data);
    Preconditions.checkNotNull(size);
    if (data != rawImageBuffer || !size.equals(rawImageSize)) {
      rawImageBuffer = data;
      rawImageSize = size;
      yuvImage = new YuvImage(rawImageBuffer, ImageFormat.NV21, size.width, size.height, null);
      rect = new Rect(0, 0, size.width, size.height);
    }

    Time currentTime = connectedNode.getCurrentTime();
    String frameId = "camera";

    sensor_msgs.CompressedImage image = imagePublisher.newMessage();
    image.getHeader().setStamp(currentTime);
    image.getHeader().setFrameId(frameId);

    image.setFormat("jpeg");
    Preconditions.checkState(yuvImage.compressToJpeg(rect, 100, stream));

//    //
//    Bitmap editedBitmap = Bitmap.createBitmap(size.width, size.height, Bitmap.Config.RGB_565);
//    byte[] aNv21Byte = yuvImage.getYuvData();
//    int[] rgbData = decodeGreyscale(aNv21Byte, size.width, size.height);
//    editedBitmap.setPixels(rgbData, 0, size.width, 0, 0, size.width, size.height);
//    editedBitmap.compress(Bitmap.CompressFormat.PNG, 100, stream);
//    //

    image.setData(stream.buffer().copy());
    stream.buffer().clear();

    imagePublisher.publish(image);

    sensor_msgs.CameraInfo cameraInfo = cameraInfoPublisher.newMessage();
    cameraInfo.getHeader().setStamp(currentTime);
    cameraInfo.getHeader().setFrameId(frameId);

    cameraInfo.setWidth(size.width);
    cameraInfo.setHeight(size.height);
    cameraInfoPublisher.publish(cameraInfo);
  }
}