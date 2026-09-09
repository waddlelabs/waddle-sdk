// SPDX-License-Identifier: Apache-2.0
// Dimensional wrapper over MuJoCo's installed first-party SDFs.
// No thread formula or contact-force implementation is duplicated here.
#include <mujoco/mujoco.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

namespace {
constexpr const char* attributes[] = {"radius", "scale"};
struct Parameters { mjtNum values[2]; };

template<int Kind> struct Metric {
  static inline const mjpPlugin* source = nullptr;
  static mjtNum Distance(const mjtNum point[3], const mjtNum* parameters) {
    const mjtNum scale = parameters[1];
    const mjtNum local[] = {point[0]/scale, point[1]/scale, point[2]/scale};
    return scale * source->sdf_staticdistance(local, parameters);
  }
  static void Attributes(mjtNum* result, const char* names[], const char* values[]) {
    source->sdf_attribute(result, names, values);
    char* end = nullptr;
    result[1] = *values[1] ? std::strtod(values[1], &end) : 1.;
    if ((end && *end) || !std::isfinite(result[1]) || result[1] <= 0) {
      mju_error("thread scale must be finite and positive");
    }
  }
  static void Register(const char* original, const char* name) {
    source = mjp_getPlugin(original, nullptr);
    if (!source || source->nattribute != 1 ||
        std::strcmp(source->attributes[0], "radius") ||
        !source->sdf_staticdistance || !source->sdf_attribute || !source->sdf_aabb) {
      std::fprintf(stderr, "metric thread needs the installed first-party %s radius API\n", original);
      return;
    }
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = name;
    plugin.capabilityflags = mjPLUGIN_SDF;
    plugin.nattribute = 2;
    plugin.attributes = attributes;
    plugin.nstate = [](const mjModel*, int) { return 0; };
    plugin.init = [](const mjModel* m, mjData* d, int instance) {
      auto* p = new(std::nothrow) Parameters;
      if (!p) return -1;
      const char* names[] = {"radius", "scale"};
      const char* values[] = {mj_getPluginConfig(m, instance, "radius"),
                              mj_getPluginConfig(m, instance, "scale")};
      Attributes(p->values, names, values);
      d->plugin_data[instance] = reinterpret_cast<uintptr_t>(p);
      return 0;
    };
    plugin.destroy = [](mjData* d, int instance) {
      delete reinterpret_cast<Parameters*>(d->plugin_data[instance]);
      d->plugin_data[instance] = 0;
    };
    plugin.copy = [](mjData* dest, const mjModel*, const mjData* src, int instance) {
      *reinterpret_cast<Parameters*>(dest->plugin_data[instance]) =
          *reinterpret_cast<const Parameters*>(src->plugin_data[instance]);
    };
    plugin.reset = [](const mjModel*, mjtNum*, void*, int) {};
    plugin.compute = [](const mjModel*, mjData*, int, int) {};
    plugin.sdf_staticdistance = Distance;
    plugin.sdf_distance = [](const mjtNum point[3], const mjData* d, int instance) {
      return Distance(point, reinterpret_cast<const Parameters*>(d->plugin_data[instance])->values);
    };
    plugin.sdf_gradient = [](mjtNum gradient[3], const mjtNum point[3], const mjData* d, int instance) {
      const auto* p = reinterpret_cast<const Parameters*>(d->plugin_data[instance]);
      // Same forward-difference step as the first-party plugin, in its units.
      const mjtNum eps = 1e-8 * p->values[1];
      const mjtNum base = Distance(point, p->values);
      for (int axis = 0; axis < 3; ++axis) {
        mjtNum next[] = {point[0], point[1], point[2]};
        next[axis] += eps;
        gradient[axis] = (Distance(next, p->values)-base)/eps;
      }
    };
    plugin.sdf_attribute = Attributes;
    plugin.sdf_aabb = [](mjtNum bounds[6], const mjtNum* p) {
      source->sdf_aabb(bounds, p);
      for (int i = 0; i < 6; ++i) bounds[i] *= p[1];
    };
    mjp_registerPlugin(&plugin);
  }
};
mjPLUGIN_LIB_INIT(metric_thread) {
  Metric<0>::Register("mujoco.sdf.nut", "waddle_sdk.sdf.metric_nut");
  Metric<1>::Register("mujoco.sdf.bolt", "waddle_sdk.sdf.metric_bolt");
}
}
