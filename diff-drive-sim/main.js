import { MOTION_MODEL, stepDifferentialDrive } from "./differential-drive.js";
import { createCanvasRenderer } from "./canvas-renderer.js";

const canvas = document.getElementById("scene");
const modeReadout = document.getElementById("modeReadout");
const poseReadout = document.getElementById("poseReadout");
const omegaRight = document.getElementById("omegaRight");
const omegaLeft = document.getElementById("omegaLeft");
const omegaRightValue = document.getElementById("omegaRightValue");
const omegaLeftValue = document.getElementById("omegaLeftValue");
const toggleMotion = document.getElementById("toggleMotion");
const resetSimulation = document.getElementById("resetSimulation");

const renderer = createCanvasRenderer(canvas);

const defaults = {
    x: 0,
    y: 0,
    theta: 0,
    omegaRight: 1.5,
    omegaLeft: 1.0,
    running: true,
};

const state = {
    x: defaults.x,
    y: defaults.y,
    theta: defaults.theta,
    omegaRight: defaults.omegaRight,
    omegaLeft: defaults.omegaLeft,
    running: defaults.running,
    history: [{ x: defaults.x, y: defaults.y }],
};

let lastFrameTime = performance.now();
let accumulator = 0;

function formatAngle(angle) {
    return `${angle.toFixed(2)} rad`;
}

function formatSpeed(value) {
    return `${Number(value).toFixed(2)} rad/s`;
}

function syncControls() {
    omegaRightValue.textContent = formatSpeed(state.omegaRight);
    omegaLeftValue.textContent = formatSpeed(state.omegaLeft);
    modeReadout.textContent = state.running ? "Running" : "Paused";
    toggleMotion.textContent = state.running ? "Pause" : "Play";
    poseReadout.textContent = `x: ${state.x.toFixed(2)}, y: ${state.y.toFixed(2)}, θ: ${formatAngle(state.theta)}`;
}

function setSliderValues() {
    omegaRight.value = String(state.omegaRight);
    omegaLeft.value = String(state.omegaLeft);
}

function resetSimulationState() {
    state.x = defaults.x;
    state.y = defaults.y;
    state.theta = defaults.theta;
    state.omegaRight = defaults.omegaRight;
    state.omegaLeft = defaults.omegaLeft;
    state.running = defaults.running;
    state.history = [{ x: state.x, y: state.y }];
    accumulator = 0;
    lastFrameTime = performance.now();
    setSliderValues();
    syncControls();
    render();
}

function updateWheelValues() {
    state.omegaRight = Number(omegaRight.value);
    state.omegaLeft = Number(omegaLeft.value);
    syncControls();
    render();
}

function resizeCanvas() {
    renderer.resizeToParent();
    render();
}

function render() {
    renderer.render({ pose: state, history: state.history });
    syncControls();
}

function frame(now) {
    const elapsed = Math.min((now - lastFrameTime) / 1000, 0.1);
    lastFrameTime = now;

    if (state.running) {
        accumulator += elapsed;

        while (accumulator >= MOTION_MODEL.dt) {
            stepDifferentialDrive(state, MOTION_MODEL.dt, MOTION_MODEL);
            accumulator -= MOTION_MODEL.dt;
        }
    } else {
        accumulator = 0;
    }

    render();
    window.requestAnimationFrame(frame);
}

omegaRight.addEventListener("input", updateWheelValues);
omegaLeft.addEventListener("input", updateWheelValues);

toggleMotion.addEventListener("click", () => {
    state.running = !state.running;
    lastFrameTime = performance.now();
    syncControls();
});

resetSimulation.addEventListener("click", resetSimulationState);

window.addEventListener("resize", resizeCanvas);
window.addEventListener("load", () => {
    setSliderValues();
    syncControls();
    resizeCanvas();
    window.requestAnimationFrame(frame);
});
