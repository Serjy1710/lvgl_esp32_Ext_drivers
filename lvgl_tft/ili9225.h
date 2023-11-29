/**
 * @file lv_templ.h
 *
 */

#ifndef ILI9225_H
#define ILI9225_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "../lvgl_helpers.h"

/*********************
 *      DEFINES
 *********************/
#define ILI9225_DC CONFIG_LV_DISP_PIN_DC
#define ILI9225_RST CONFIG_LV_DISP_PIN_RST
#define ILI9225_BCKL CONFIG_LV_DISP_PIN_BCKL

#define ILI9225_ENABLE_BACKLIGHT_CONTROL CONFIG_LV_ENABLE_BACKLIGHT_CONTROL

#if CONFIG_LV_BACKLIGHT_ACTIVE_LVL
#define ILI9225_BCKL_ACTIVE_LVL 1
#else
#define ILI9225_BCKL_ACTIVE_LVL 0
#endif

#define ILI9225_INVERT_COLORS CONFIG_LV_INVERT_COLORS
#define ILI9225_DISPLAY_ORIENTATION CONFIG_LV_DISPLAY_ORIENTATION

#define ILI9225_TFTWIDTH 176  ///< ILI9225 max TFT width
#define ILI9225_TFTHEIGHT 220 ///< ILI9225 max TFT height

  /* Maximal horizontal and vertical resolution to support by the library.



  #if (CONFIG_LV_DISPLAY_ORIENTATION & 0x01)
  #    define LV_HOR_RES_MAX ILI9225_TFTWIDTH
  #    define LV_VER_RES_MAX ILI9225_TFTHEIGHT
  #  else
  #    define  LV_HOR_RES_MAX ILI9225_TFTHEIGHT
  #    define  LV_VER_RES_MAX ILI9225_TFTWIDTH
  #  endif

  #define  LV_HOR_RES_MAX          ILI9225_TFTHEIGHT
  #define  LV_VER_RES_MAX          ILI9225_TFTWIDTH
  */

  /**********************
   *      TYPEDEFS
   **********************/

  // Swap any type
  // template <typename T> static inline void
  // swap_coord(T& a, T& b) { T t = a; a = b; b = t; }

  // static inline void swap_coord(int16_t & a, int16_t & b) { int16_t t = a; a = b; b = t; }

  /**********************
   * GLOBAL PROTOTYPES
   **********************/

  void ili9225_init(void);
  void ili9225_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
  void ili9225_enable_backlight(bool backlight);
  void ili9225_sleep_in(void);
  void ili9225_sleep_out(void);
  void drawPixel(int16_t x, int16_t y, uint16_t color);

  /**********************
   *      MACROS
   **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ILI9225_H*/
