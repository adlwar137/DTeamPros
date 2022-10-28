template<typename T>
T mathy_distance_between_points(T firstX, T secondX, T firstY, T secondY) {
  return (T)sqrt(pow(firstX - secondX, 2) + pow(firstY - secondY, 2));
}

template<typename T>
T mathy_to_degrees(T angle) {
  return (T)(angle/M_PI) * 180;
}

template<typename T>
T mathy_to_radians(T angle) {
  return (T)(angle/180) * M_PI;
}

template<typename T>
T mathy_clamp(T value, T min, T max) {
  if(value > max) {
    value = max;
  }
  if(value < min) {
    value = min;
  }
  return value;
}

template<typename T>
T mathy_min(T x1, T x2) {
  if(x1 < x2) {
    return x1;
  } else {
    return x2;
  }
}

template<typename T>
T mathy_max(T x1, T x2) {
  if(x1 > x2) {
    return x1;
  } else {
    return x2;
  }
}

template<typename T>
T mathy_remap(T value, T from1, T to1, T from2, T to2) {
  return (T)((double)(value - from1) / (to1 - from1) * (to2 - from2) + from2);
}