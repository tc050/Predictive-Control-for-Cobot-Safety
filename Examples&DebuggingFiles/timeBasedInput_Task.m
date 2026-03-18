function stateDot = timeBasedInput_Task(MotionModel, timeInterval, eeInit, eeRef, T, state)
    [refPose, refVel] = transformtraj(eeInit, eeRef, timeInterval, T);
    stateDot = derivative(MotionModel, state, refPose, refVel);
end

