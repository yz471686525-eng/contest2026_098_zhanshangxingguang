/****************************************************************************
 * app/camera/camera_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arch/board/bk7258_camera.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * camera_main
 ****************************************************************************/

int main(int argc, char *argv[])
{
  if (argc < 2)
    {
      printf("Usage: camera <command>\n");
      printf("Commands:\n");
      printf("  id        — read GC2145 chip ID\n");
      printf("  io        — DVP pin mux config/deconfig/readback\n");
      printf("  init      — GC2145 YUYV 640x480 init (MCLK stays on)\n");
      printf("  stop      — MCLK off, power off, restore pins\n");
      printf("  buf       — PSRAM framebuf alloc + test pattern verify\n");
      printf("  sync      — DVP controller + VSYNC interrupt counting\n");
      printf("  grab      — DMA single frame capture to PSRAM\n");
      printf("  dump      — capture + hexdump + 4-format YUV analysis\n");
      printf("  stream [n] — continuous capture (default 30, max 1000)\n");
      printf("  preview [n] — capture + LCD display (default 100)\n");
      printf("  testpat — color bars to LCD (no camera, tests blit)\n");
      printf("  bench  — PSRAM vs SRAM conversion timing (A/B test)\n");
      printf("  uvhist [n] [lo] [hi] — UV+Y histogram"
             " (zoom: 32 buckets in [lo,hi])\n");
      printf("  detect [n] — skin-tone direction"
             " (default 1 frame)\n");
      printf("  track [n] [inv] — skin gaze tracking"
             " (n=100 default, n=0 continuous, inv=1/0)\n");
      return 1;
    }

  if (strcmp(argv[1], "id") == 0)
    {
      return bk7258_camera_id();
    }
  else if (strcmp(argv[1], "io") == 0)
    {
      return bk7258_camera_io();
    }
  else if (strcmp(argv[1], "init") == 0)
    {
      return bk7258_camera_init();
    }
  else if (strcmp(argv[1], "stop") == 0)
    {
      return bk7258_camera_stop();
    }
  else if (strcmp(argv[1], "buf") == 0)
    {
      return bk7258_camera_buf();
    }
  else if (strcmp(argv[1], "sync") == 0)
    {
      return bk7258_camera_sync();
    }
  else if (strcmp(argv[1], "grab") == 0)
    {
      return bk7258_camera_grab();
    }
  else if (strcmp(argv[1], "dump") == 0)
    {
      return bk7258_camera_dump();
    }
  else if (strcmp(argv[1], "stream") == 0)
    {
      int n = 30;
      if (argc >= 3) n = atoi(argv[2]);
      return bk7258_camera_stream(n);
    }
  else if (strcmp(argv[1], "testpat") == 0)
    {
      return bk7258_camera_testpat();
    }
  else if (strcmp(argv[1], "preview") == 0)
    {
      int n = 100;
      int panel = 0;
      if (argc >= 3) n = atoi(argv[2]);
      if (argc >= 4) panel = atoi(argv[3]);
      return bk7258_camera_preview(n, panel);
    }
  else if (strcmp(argv[1], "bench") == 0)
    {
      return bk7258_camera_bench();
    }
  else if (strcmp(argv[1], "uvhist") == 0)
    {
      int n = 1;
      int lo = -1;
      int hi = -1;
      if (argc >= 3) n = atoi(argv[2]);
      if (argc >= 4) lo = atoi(argv[3]);
      if (argc >= 5) hi = atoi(argv[4]);
      return bk7258_camera_uvhist(n, lo, hi);
    }
  else if (strcmp(argv[1], "detect") == 0)
    {
      int n = 1;
      if (argc >= 3) n = atoi(argv[2]);
      return bk7258_camera_detect(n);
    }
  else if (strcmp(argv[1], "track") == 0)
    {
      int n = 100;
      int inv = 1;  /* default: mirror-correct "look toward you" */
      if (argc >= 3) n = atoi(argv[2]);
      if (argc >= 4) inv = atoi(argv[3]);
      return bk7258_camera_track(n, inv);
    }
  else
    {
      printf("Unknown command: %s\n", argv[1]);
      return 1;
    }
}
