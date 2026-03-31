function stateDot = timeBasedInput_Robot(motionModel, timeStep, targetStates, t, state)
    targetState_interp = interp1(timeStep, targetStates, t);
    stateDot = derivative(motionModel, state, targetState_interp);
end