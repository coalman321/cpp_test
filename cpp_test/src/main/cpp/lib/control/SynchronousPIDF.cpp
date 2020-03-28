/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/control/SynchronousPIDF.h"

void SynchronousPIDF::reset(){
    calculationMutex.lock();
    error = lastError = derivative = integral = setPoint = 0;
    continuous = enabled = false;
    calculationMutex.unlock();
}

void SynchronousPIDF::updateSetPoint(double set){
    calculationMutex.lock();
    setPoint = set;
    calculationMutex.unlock();
}

std::vector<double> SynchronousPIDF::getPIF(){
    std::vector<double> out;
    calculationMutex.lock();

    out.push_back(kp);
    out.push_back(ki);
    out.push_back(kd);
    out.push_back(kf);

    calculationMutex.unlock();
    
    return out;
}

void SynchronousPIDF::setPIDF(double kP, double kI, double kD, double kF){
    calculationMutex.lock();

    kp = kP;
    ki = kI;
    kd = kD;
    kf = kF;

    calculationMutex.unlock();
}

double SynchronousPIDF::update(double input){
    if (!enabled) {
        return 0.0;
    }

    double result;

    calculationMutex.lock();
    // calculate p term
    error = getContinuousError(setPoint - input);

    // calculate D term based on previous P term
    derivative = (error - lastError) / timeDelta;

    // calculate and bound I term to iMax
    integral += error * timeDelta;
    if (std::abs(integral) > iMax) {
        integral = std::signbit(integral)? 1 : -1 * iMax;
    }

    // save last error for d term
    lastError = error;

    result = error * kp + integral * ki + derivative * kd;

    calculationMutex.unlock();

    return clamp(result, -1, 1);
}

void SynchronousPIDF::setInputRange(double lower, double upper){
    if (upper < lower) {
        return;
    }
    calculationMutex.lock();

    inputRange = upper - lower;

    calculationMutex.unlock();
}

double SynchronousPIDF::getContinuousError(double error){
    if (continuous && inputRange > 0) {
        error = std::fmod(error, inputRange);
        if (std::abs(error) > inputRange / 2) {
            if (error > 0) {
                return error - inputRange;
            } else {
                return error + inputRange;
            }
        }
    }

    return error;
}

double SynchronousPIDF::getError(){
    double err;
    calculationMutex.lock();

    err = error;

    calculationMutex.unlock();
    return err;
}

void SynchronousPIDF::setContinuous(bool cont){
    calculationMutex.lock();

    continuous = cont;

    calculationMutex.unlock();
}

double SynchronousPIDF::clamp(double input, double lower, double upper){
    return std::max(lower, std::min(input, upper));
}