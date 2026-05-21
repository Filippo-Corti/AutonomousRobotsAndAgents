export const MOTION_MODEL = {
    wheelRadius: 0.18,
    axleLength: 0.72,
    dt: 0.05,
    maxHistory: 3000,
};

// Treat values this small as straight motion so nearly identical wheel speeds do not
// trigger a numerically unstable divide-by-nearly-zero turn-radius calculation.
export const ROTATION_EPSILON = 1e-9;

export function normalizeAngle(angle) {
    let wrapped = angle;

    while (wrapped <= -Math.PI) {
        wrapped += Math.PI * 2;
    }

    while (wrapped > Math.PI) {
        wrapped -= Math.PI * 2;
    }

    return wrapped;
}

export function stepDifferentialDrive(state, dt, motionModel = MOTION_MODEL) {
    const linearVelocity = (motionModel.wheelRadius * (state.omegaRight + state.omegaLeft)) / 2;
    const angularVelocity = (motionModel.wheelRadius * (state.omegaRight - state.omegaLeft)) / motionModel.axleLength;

    const thetaNext = state.theta + angularVelocity * dt;
    const radius = linearVelocity / angularVelocity;

    const x_dot = linearVelocity * Math.cos(state.theta);
    const y_dot = linearVelocity * Math.sin(state.theta);
    const theta_dot = angularVelocity;

    state.x += x_dot * dt;
    state.y += y_dot * dt;
    state.theta += theta_dot * dt;

    state.theta = normalizeAngle(state.theta);
    state.history.push({ x: state.x, y: state.y });

    if (state.history.length > motionModel.maxHistory) {
        state.history.splice(0, state.history.length - motionModel.maxHistory);
    }
}