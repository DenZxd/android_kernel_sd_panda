#ifndef _MEDIA_OV2655_H
#define _MEDIA_OV2655_H

#include <media/v4l2-subdev.h>

struct ov2655_platform_data {
      int (*s_power)(struct v4l2_subdev *subdev, int on);
};

#endif
