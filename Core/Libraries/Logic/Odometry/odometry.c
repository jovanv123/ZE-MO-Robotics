#include "odometry.h"

static float dist_per_tick_r;
static float dist_per_tick_l;
static float wheel_base;

static float x = 0.0f, y = 0.0f, fi = 0.0f;
static int32_t last_n_r = 0;
static int32_t last_n_l = 0;
static float ds_r = 0.0f, ds_l = 0.0f;

void odometry_init(float d_r, float d_l, float base) {
    wheel_base = base;
    dist_per_tick_r = (d_r * M_PI) / (2048.0f * 4.0f);
    dist_per_tick_l = -(d_l * M_PI) / (2048.0f * 4.0f);

    last_n_r = encoder_get_count_right_motor();
    last_n_l = encoder_get_count_left_motor();
}

void calculate_odometry() {
    int32_t current_n_r = encoder_get_count_right_motor();
    int32_t current_n_l = encoder_get_count_left_motor();

    int32_t dn_r = current_n_r - last_n_r;
    int32_t dn_l = current_n_l - last_n_l;

    ds_r = (float)dn_r * dist_per_tick_r;
    ds_l = (float)dn_l * dist_per_tick_l;

    last_n_r = current_n_r;
    last_n_l = current_n_l;

    float ds = (ds_r + ds_l) / 2.0f;
    float dfi = (ds_r - ds_l) / wheel_base;
    float fi_mid = fi + (dfi / 2.0f);

    x += ds * cosf(fi_mid);
    y += ds * sinf(fi_mid);
    fi += dfi;

    if (fi > M_PI)  fi -= 2.0f * M_PI;
    if (fi < -M_PI) fi += 2.0f * M_PI;

}

float get_speed_r() {
    return ds_r * 1000.0f;
}

float get_speed_l() {
    return ds_l * 1000.0f;
}

float get_x() {
    return x;
}

float get_y() {
    return y;
}

float get_fi() {
    return fi;
}

void set_x_y(float x_p, float y_p, float fi_p) {
    x = x_p;
    y = y_p;
    fi = fi_p;
}
