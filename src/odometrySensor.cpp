#include "odometrySensor.h"

OdometrySensor::OdometrySensor(pros::ADIEncoder* left_encoder, pros::ADIEncoder* right_encoder, pros::ADIEncoder* strafe_encoder) {
  this->left_encoder = left_encoder;
  this->right_encoder = right_encoder;
  this->strafe_encoder = strafe_encoder;
}

void OdometrySensor::updatePosition() {
  double leftDistance = OdometrySensor::getEncoderDistance(this->left_encoder, TRACKING_WHEEL_DIAMETER);
  double rightDistance = OdometrySensor::getEncoderDistance(this->right_encoder, TRACKING_WHEEL_DIAMETER);
  double strafeDistance = OdometrySensor::getEncoderDistance(this->strafe_encoder, TRACKING_WHEEL_DIAMETER);

  double prevLeftDistance;
  double prevRightDistance;
  double prevStrafeDistance;
  double prevHeading;

  double deltaLeftDistance = leftDistance - prevLeftDistance;
  double deltaRightDistance = rightDistance - prevRightDistance;
  double deltaStrafeDistance = strafeDistance - prevStrafeDistance;

  double deltaHeading = (deltaLeftDistance - deltaRightDistance) / (LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER);
  double heading = heading + deltaHeading;

  double deltaXPosition, deltaYPosition;
  double deltaGlobalXPosition, deltaGlobalYPosition;

  if(deltaHeading <= 0.000001 && deltaHeading >= -0.000001) {
    deltaXPosition = deltaStrafeDistance;
    deltaYPosition = deltaRightDistance;
  } else {
    deltaXPosition = 2*sin(deltaHeading/2)*(((deltaStrafeDistance/deltaHeading) + STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
    deltaYPosition = 2*sin(deltaHeading/2)*(((deltaRightDistance/deltaHeading) + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
  }

  double averageHeading = prevHeading + (deltaHeading/2);

  deltaGlobalXPosition = deltaXPosition*cos(-averageHeading) - deltaYPosition*sin(-averageHeading);
  deltaGlobalYPosition = deltaXPosition*sin(-averageHeading) + deltaYPosition*cos(-averageHeading);

  double GlobalXPosition = GlobalXPosition + deltaGlobalXPosition;
  double GlobalYPosition = GlobalYPosition + deltaGlobalYPosition;

  prevLeftDistance = leftDistance;
  prevRightDistance = rightDistance;
  prevStrafeDistance = strafeDistance;
  prevHeading = heading;

  this->pose.x = GlobalXPosition;
  this->pose.y = GlobalYPosition;
  this->pose.heading = heading;
}

void OdometrySensor::updatePosition(double heading) {
  double leftDistance = OdometrySensor::getEncoderDistance(this->left_encoder, TRACKING_WHEEL_DIAMETER);
  double rightDistance = OdometrySensor::getEncoderDistance(this->right_encoder, TRACKING_WHEEL_DIAMETER);
  double strafeDistance = OdometrySensor::getEncoderDistance(this->strafe_encoder, TRACKING_WHEEL_DIAMETER);

  double prevLeftDistance;
  double prevRightDistance;
  double prevStrafeDistance;
  double prevHeading;

  double deltaLeftDistance = leftDistance - prevLeftDistance;
  double deltaRightDistance = rightDistance - prevRightDistance;
  double deltaStrafeDistance = strafeDistance - prevStrafeDistance;

  double deltaHeading = heading - prevHeading;

  double deltaXPosition, deltaYPosition;
  double deltaGlobalXPosition, deltaGlobalYPosition;

  if(deltaHeading <= 0.000001 && deltaHeading >= -0.000001) {
    deltaXPosition = deltaStrafeDistance;
    deltaYPosition = deltaRightDistance;
  } else {
    deltaXPosition = 2*sin(deltaHeading/2)*(((deltaStrafeDistance/deltaHeading) + STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
    deltaYPosition = 2*sin(deltaHeading/2)*(((deltaRightDistance/deltaHeading) + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
  }

  double averageHeading = prevHeading + (deltaHeading/2);

  deltaGlobalXPosition = deltaXPosition*cos(-averageHeading) - deltaYPosition*sin(-averageHeading);
  deltaGlobalYPosition = deltaYPosition*sin(-averageHeading) + deltaYPosition*cos(-averageHeading);

  double GlobalXPosition = GlobalXPosition + deltaGlobalXPosition;
  double GlobalYPosition = GlobalYPosition + deltaGlobalYPosition;

  prevLeftDistance = leftDistance;
  prevRightDistance = rightDistance;
  prevStrafeDistance = strafeDistance;
  prevHeading = heading;

  this->pose.x = GlobalXPosition;
  this->pose.y = GlobalYPosition;
  this->pose.heading = heading;
}

int32_t OdometrySensor::trackPosition(int32_t interval) {
  while(1) {
    updatePosition();
    pros::delay(interval);
  }
}

int32_t OdometrySensor::trackPosition(int32_t interval, double* heading) {
  while(1) {
    updatePosition(*heading);
    pros::delay(interval);
  }
}

Vector OdometrySensor::getPosition() {
  Vector pos;
  pos.x = this->pose.x;
  pos.y = this->pose.y;
  return pos;
}

double OdometrySensor::getHeading() {
  return this->pose.heading;
}

Vector OdometrySensor::getStatus() {
  Vector pos;
  pos.x = this->pose.x;
  pos.y = this->pose.y;
  pos.heading = this->pose.heading;
  return pos;
}

void OdometrySensor::reset() {
  this->pose.x = 0;
  this->pose.y = 0;
  this->pose.heading = 0;
}

double OdometrySensor::getEncoderDistance(pros::ADIEncoder* encoder, double wheelDiameter) {
  return ((double)encoder->get_value() / 360) * M_PI * wheelDiameter;
}
/*
double odometry_get_encoder_distance(adi_encoder_t encoder, double wheel_diameter) {
  return ((double)(ext_adi_encoder_get(encoder)) / 360) * M_PI * wheel_diameter;
}

void odometry_track(void* tracking_params) {
  tracking_params_t *paramsP = (tracking_params_t*)tracking_params;
  tracking_params_t params = *paramsP;
  while(1) {
    double leftDistance = odometry_get_encoder_distance(params.left_encoder, TRACKING_WHEEL_DIAMETER);
    double rightDistance = odometry_get_encoder_distance(params.right_encoder, TRACKING_WHEEL_DIAMETER);
    double strafeDistance = odometry_get_encoder_distance(params.strafe_encoder, TRACKING_WHEEL_DIAMETER);


    double prevLeftDistance;
    double prevRightDistance;
    double prevStrafeDistance;
    double prevHeading;

    double deltaLeftDistance = leftDistance - prevLeftDistance;
    double deltaRightDistance = rightDistance - prevRightDistance;
    double deltaStrafeDistance = strafeDistance - prevStrafeDistance;

    //radians

    //not calculating delta w
    //double deltaHeading = (deltaLeftDistance - deltaRightDistance) / (LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER);
    //printf("currentHeading: %f\n", deltaHeading);
    //double heading = heading + deltaHeading;
    double imu_heading = imu_get_heading(13);
    double heading;
    if(imu_get_status(13) == E_IMU_STATUS_CALIBRATING) {
      heading = 0;
    }
    if (imu_heading <= 0.01 && imu_heading >= -0.01) {
      heading = 0;
    } else {
      heading = (imu_get_heading(13) / 360) * 2 * M_PI; 
    }
    double deltaHeading = heading - prevHeading;

    double deltaXPosition, deltaYPosition;
    double deltaGlobalXPosition, deltaGlobalYPosition;
    
    //printf("leftEncoder: %f, RightEncoder: %f, leftRaw: %d, rightRaw: %d, Heading: %f\n", leftDistance, rightDistance, adi_encoder_get(params.left_encoder), adi_encoder_get(params.right_encoder),mathy_to_degrees(heading));

    if(deltaHeading <= 0.000001 && deltaHeading >= -0.000001) {
      deltaXPosition = deltaStrafeDistance;
      deltaYPosition = deltaRightDistance;
    } else {
      deltaXPosition = 2*sin(deltaHeading/2)*(((deltaStrafeDistance/deltaHeading) + STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
      deltaYPosition = 2*sin(deltaHeading/2)*(((deltaRightDistance/deltaHeading) + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
    }

    double averageHeading = prevHeading + (deltaHeading/2);

    deltaGlobalXPosition = deltaXPosition*cos(-averageHeading) - deltaYPosition*sin(-averageHeading);
    deltaGlobalYPosition = deltaXPosition*sin(-averageHeading) + deltaYPosition*cos(-averageHeading);

    double GlobalXPosition = GlobalXPosition + deltaGlobalXPosition;
    double GlobalYPosition = GlobalYPosition + deltaGlobalYPosition;

    prevLeftDistance = leftDistance;
    prevRightDistance = rightDistance;
    prevStrafeDistance = strafeDistance;
    prevHeading = heading;

    params.pose->x = GlobalXPosition;
    params.pose->y = GlobalYPosition;
    params.pose->w = heading;

    //wait to free up resources
    delay(20);
  }
}
*/