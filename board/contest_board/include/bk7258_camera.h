/****************************************************************************
 * board/contest_board/src/bk7258_camera.h
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

#ifndef __BOARD_CONTEST_BOARD_SRC_BK7258_CAMERA_H
#define __BOARD_CONTEST_BOARD_SRC_BK7258_CAMERA_H

#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bk7258_camera_id
 *
 * Description:
 *   Phase 0 "camera id" command.  Reads GC2145 registers 0xF0 and 0xF1
 *   via open-drain SCCB bit-bang and prints the chip ID.
 *
 *   Safety: checks all board GPIO mappings first.  If any required pin
 *   is unconfigured, exits without touching any hardware register.
 *
 * Returns:
 *   0 on success (ID matches 0x2145), negative errno on failure.
 *
 ****************************************************************************/

int bk7258_camera_id(void);

/****************************************************************************
 * Name: bk7258_camera_io
 *
 * Description:
 *   Phase 1 Step 1: configure DVP data pins P29-P39, readback verify,
 *   deconfigure, readback verify restore.
 *
 * Returns:
 *   0 on success, negative errno on failure.
 *
 ****************************************************************************/

int bk7258_camera_io(void);

/****************************************************************************
 * Name: bk7258_camera_init
 *
 * Description:
 *   Phase 1 Step 2: power on, MCLK, reset, read chip ID, write GC2145
 *   init table (640x480), readback verify.
 *   MCLK and power are left ON after success (for oscilloscope verify).
 *
 * Returns:
 *   0 on success, negative errno on failure.
 *
 ****************************************************************************/

int bk7258_camera_init(void);

/****************************************************************************
 * Name: bk7258_camera_stop
 *
 * Description:
 *   Turn off MCLK, power off, restore DVP pins, free frame buffers.
 *
 * Returns:
 *   0 on success.
 *
 ****************************************************************************/

int bk7258_camera_stop(void);

/****************************************************************************
 * Name: bk7258_camera_buf
 *
 * Description:
 *   Phase 1 Step 3: init PSRAM, allocate two frame buffers at 0x60000000,
 *   write test pattern to buf[0], sampled readback verify.
 *
 * Returns:
 *   0 on success, negative errno on failure.
 *
 ****************************************************************************/

int bk7258_camera_buf(void);

/****************************************************************************
 * Name: bk7258_camera_dvp_active
 *
 * Description:
 *   Runtime flag: returns true if DVP data pins (P29-P39) are currently
 *   configured.  Used by lcdtest for mutual exclusion.
 *
 * Returns:
 *   true if DVP pins are configured, false otherwise.
 *
 ****************************************************************************/

bool bk7258_camera_dvp_active(void);

/****************************************************************************
 * Name: bk7258_camera_sync
 *
 * Description:
 *   Phase 1 Round 2 Step 4a: DVP controller config + interrupt counting.
 *   Configures CIS controller, attaches VSYNC/YUV_ARRIVED ISR, waits
 *   for 10 VSYNC edges (5s timeout).  Restores all on exit.
 *
 * Prerequisites:
 *   camera init (MCLK on, GC2145 configured) + camera buf (PSRAM ready).
 *
 * Returns:
 *   0 on success (VSYNC edges detected), negative errno on failure.
 *
 ****************************************************************************/

int bk7258_camera_sync(void);

/****************************************************************************
 * Name: bk7258_camera_grab
 *
 * Description:
 *   Phase 1 Round 2 Step 4b: DMA single frame capture to PSRAM.
 *   Fills buf[0] with 0x5A, configures CIS controller, waits for one
 *   complete frame (YUV_ARRIVED), hexdumps first 256 bytes.  Restores
 *   all on exit.
 *
 * Prerequisites:
 *   camera init + camera buf.
 *
 * Returns:
 *   0 on success (frame captured), negative errno on failure.
 *
 ****************************************************************************/

int bk7258_camera_grab(void);

int bk7258_camera_stream(int n_frames);
int bk7258_camera_preview(int n_frames, int panel);
int bk7258_camera_dump(void);
int bk7258_camera_testpat(void);
int bk7258_camera_bench(void);
int bk7258_camera_uvhist(int n_frames, int zoom_lo, int zoom_hi);
int bk7258_camera_detect(int n_frames);
int bk7258_camera_track(int n_frames, int invert);

void bk7258_lcd_blit_rgb565(int panel,
                             uint16_t x0, uint16_t y0,
                             uint16_t w, uint16_t h,
                             const uint8_t *rgb565);
void bk7258_lcd_eye_draw(int panel, int gaze_dx);
void bk7258_lcd_eye_gaze(int panel, int old_dx, int new_dx);
int bk7258_lcd_preview_init(void);
void bk7258_lcd_preview_deinit(void);

#endif /* __BOARD_CONTEST_BOARD_SRC_BK7258_CAMERA_H */
