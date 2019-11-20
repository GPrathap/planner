// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _UTILS_RADIANS_H_
#define _UTILS_RADIANS_H_

#include <math.h>
#include <stdio.h>
#include <limits>

namespace kamaz {
  namespace hagen{
class Radians;
  }
}

// this class provides user defined literals.
// https://akrzemi1.wordpress.com/2012/08/12/user-defined-literals-part-i/
// forward declare Radians class and literals
constexpr kamaz::hagen::Radians operator"" _rad(long double angle);
constexpr kamaz::hagen::Radians operator"" _deg(
    unsigned long long int angle);
constexpr kamaz::hagen::Radians operator"" _deg(long double angle);

namespace kamaz {

namespace hagen{
class Radians {
 public:
  class IsRadians {};  // a tag to prevent using raw constructor
  Radians() : _raw_angle{0}, _valid{false} {}
  explicit constexpr Radians(IsRadians, double angle)
      : _raw_angle{angle}, _valid{true} {}

  inline double val() const { return _raw_angle; }
  inline bool valid() const { return _valid; }

  static Radians FromRadians(double radians) {
    return Radians{IsRadians{}, radians};
  }

  static Radians FromDegrees(double angle) {
    return Radians{IsRadians{}, static_cast<double>(angle * M_PI / 180.)};
  }

  double ToDegrees() const { return 180. * _raw_angle / M_PI; }

  Radians operator-(const Radians& other) const {
    return FromRadians(_raw_angle - other._raw_angle);
  }

  Radians operator+(const Radians& other) const {
    return FromRadians(_raw_angle + other._raw_angle);
  }

  Radians operator+=(const Radians& other) {
    this->_raw_angle += other._raw_angle;
    return *this;
  }

  Radians operator-=(const Radians& other) {
    this->_raw_angle -= other._raw_angle;
    return *this;
  }

  Radians operator/(const double& num) const {
    return FromRadians(_raw_angle / num);
  }

  double operator/(const Radians& other) const {
    return _raw_angle / other._raw_angle;
  }

  Radians operator*(const double& num) const {
    return FromRadians(_raw_angle * num);
  }

  Radians operator-() { return FromRadians(-_raw_angle); }

  bool operator<(const Radians& other) const {
    return _raw_angle <
           other._raw_angle - std::numeric_limits<double>::epsilon();
  }
  bool operator>(const Radians& other) const {
    return _raw_angle >
           other._raw_angle + std::numeric_limits<double>::epsilon();
  }

  void Normalize(const Radians& from = 0_deg, const Radians& to = 360_deg) {
    double diff = (to - from).val();
    while (_raw_angle < from.val()) {
      _raw_angle += diff;
    }
    while (_raw_angle > to.val()) {
      _raw_angle -= diff;
    }
  }

  Radians Normalize(const Radians& from = 0_deg,
                    const Radians& to = 360_deg) const {
    Radians new_angle = FromRadians(_raw_angle);
    new_angle.Normalize(from, to);
    return new_angle;
  }

  static Radians Abs(const Radians& angle) {
    return Radians::FromRadians(fabs(angle._raw_angle));
  }

  static Radians Floor(const Radians& angle) {
    return Radians::FromDegrees(floor(angle.ToDegrees()));
  }

  double _raw_angle;
 protected:
  bool _valid;
};
}
}  

constexpr kamaz::hagen::Radians operator"" _rad(long double angle) {
  return kamaz::hagen::Radians{kamaz::hagen::Radians::IsRadians{},
                                   static_cast<double>(angle)};
}

constexpr kamaz::hagen::Radians operator"" _deg(
    unsigned long long int angle) {
  return kamaz::hagen::Radians{kamaz::hagen::Radians::IsRadians{},
                                   static_cast<double>(angle * M_PI / 180.0)};
}

constexpr kamaz::hagen::Radians operator"" _deg(long double angle) {
  return kamaz::hagen::Radians{kamaz::hagen::Radians::IsRadians{},
                                   static_cast<double>(angle * M_PI / 180.0)};
}

#endif 
