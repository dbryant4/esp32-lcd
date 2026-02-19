#ifndef LV_CONF_H
#define LV_CONF_H

/* 1: Enable the content of this file */
#define LV_CONF_SKIP 0

/* Color depth of the generated content */
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

/* Tick source provided by millis() callback in sketch */
#define LV_TICK_CUSTOM 1

/* Memory settings */
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (64U * 1024U)

/* Font defaults */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_36 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/* Keep logging off for lower overhead */
#define LV_USE_LOG 0

/* Chart widget for 24-hour trend display */
#define LV_USE_CHART 1

#endif /*LV_CONF_H*/

