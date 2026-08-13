// @ts-check

const canvas = /** @type {HTMLCanvasElement} */
               (document.getElementById('simCanvas'));

const ctx = canvas.getContext('2d');

const sliderKp = /** @type {HTMLInputElement} */
                 (document.getElementById('sliderKp'));

const sliderKd = /** @type {HTMLInputElement} */
                 (document.getElementById('sliderKd'));

const scrubber = /** @type {HTMLInputElement} */
                 (document.getElementById('scrubber'));

const valKp = document.getElementById('valKp');
const valKd = document.getElementById('valKd');
const btnPlay = document.getElementById('btnPlay');
const btnReset = document.getElementById('btnReset');
const timeReadout = document.getElementById('timeReadout');

// Physical & Simulation Constants
const m = 1.0;            // Mass (kg) / Inertia (I)
const totalTime = 60.0;   // 1 minute
const dt = 0.01;          // Time step (100 Hz simulation resolution)
const totalSteps = Math.floor(totalTime / dt);
const setpoint = 1.0;

let Kp = parseFloat(sliderKp.value);
let Kd = parseFloat(sliderKd.value);
let isPlaying = true;
let currentStep = 0;

// Float tracker to allow for fractional playback speeds
let exactStep = 0;
const playbackSpeed = 0.1; // 0.5 = Half speed. Change to 0.25 for quarter speed, etc.

// Pre-allocated array for fixed time window trajectories
let trajectory = new Array(totalSteps);

function resizeCanvas() {
    const rect = canvas.parentElement.getBoundingClientRect();
    canvas.width = rect.width * window.devicePixelRatio;
    canvas.height = rect.height * window.devicePixelRatio;
    ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Recompute full trajectory via RK4 whenever parameters change
function recomputeTrajectory() {
    let pos = 0.0;
    let vel = 0.0;

    for (let i = 0; i < totalSteps; i++) {
        const e = setpoint - pos;
        const Fp = Kp * e;
        const Fd = -Kd * vel;
        const Ftotal = Fp + Fd;

        // Store state and dynamic forces for visualization
        trajectory[i] = { p: pos, v: vel, Fp, Fd, Ftotal };

        // RK4 Numerical Integration Step
        const derivatives = (/** @type {number} */ p,
                             /** @type {number} */ v) => {
            const err = setpoint - p;
            const accel = (Kp * err - Kd * v) / m;
            return { dPos: v, dVel: accel };
        };

        const k1 = derivatives(pos, vel);
        const k2 = derivatives(pos + 0.5 * dt * k1.dPos, vel + 0.5 * dt * k1.dVel);
        const k3 = derivatives(pos + 0.5 * dt * k2.dPos, vel + 0.5 * dt * k2.dVel);
        const k4 = derivatives(pos + dt * k3.dPos, vel + dt * k3.dVel);

        pos += (dt / 6) * (k1.dPos + 2 * k2.dPos + 2 * k3.dPos + k4.dPos);
        vel += (dt / 6) * (k1.dVel + 2 * k2.dVel + 2 * k3.dVel + k4.dVel);
    }
}

// UI Event Listeners
sliderKp.addEventListener('input', (e) => {
    const target = /** @type {HTMLInputElement} */ (e.target);
    Kp = parseFloat(target.value);
    valKp.textContent = Kp.toFixed(1);
    recomputeTrajectory();
});

sliderKd.addEventListener('input', (e) => {
    const target = /** @type {HTMLInputElement} */ (e.target);
    Kd = parseFloat(target.value);
    valKd.textContent = Kd.toFixed(1);
    recomputeTrajectory();
});

scrubber.addEventListener('input', (e) => {
    const target = /** @type {HTMLInputElement} */ (e.target);
    const progress = parseFloat(target.value) / 1000;
    currentStep = Math.floor(progress * (totalSteps - 1));
    exactStep = currentStep; // Sync float tracker
    isPlaying = false;
    btnPlay.textContent = 'Play';
});

btnPlay.addEventListener('click', () => {
    isPlaying = !isPlaying;
    btnPlay.textContent = isPlaying ? 'Pause' : 'Play';
    if (isPlaying && currentStep >= totalSteps - 1) {
        currentStep = 0;
        exactStep = 0;
    }
});

btnReset.addEventListener('click', () => {
    currentStep = 0;
    exactStep = 0;
    isPlaying = true;
    btnPlay.textContent = 'Pause';
});

// Drawing Helpers for Physical Models
function drawSpring(/** @type {CanvasRenderingContext2D} */ ctx,
                    /** @type {number} */ x,
                    /** @type {number} */ yTop,
                    /** @type {number} */ yBottom) {
    const coils = 12;
    const w = 15;
    const pad = 10;
    const startY = yTop + pad;
    const endY = yBottom - pad;
    const dy = (endY - startY) / coils;

    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    ctx.moveTo(x, yTop);
    ctx.lineTo(x, startY);

    for (let i = 0; i < coils; i++) {
        const sign = (i % 2 === 0) ? 1 : -1;
        ctx.lineTo(x + sign * w, startY + i * dy + dy / 2);
    }
    ctx.lineTo(x, endY);
    ctx.lineTo(x, yBottom);
    ctx.stroke();
}

function drawPendulum(/** @type {CanvasRenderingContext2D} */ ctx,
                      /** @type {number} */ pivotX,
                      /** @type {number} */ pivotY,
                      /** @type {number} */ errorPercent,
                      /** @type {number} */ length) {
    const angle = -errorPercent * (Math.PI / 2.5);

    ctx.strokeStyle = '#ef4444';
    ctx.setLineDash([4, 4]);
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(pivotX, pivotY);
    ctx.lineTo(pivotX, pivotY + length + 25);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = '#94a3b8';
    ctx.beginPath();
    ctx.arc(pivotX, pivotY, 5, 0, Math.PI * 2);
    ctx.fill();

    const bobX = pivotX + length * Math.sin(angle);
    const bobY = pivotY + length * Math.cos(angle);

    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(pivotX, pivotY);
    ctx.lineTo(bobX, bobY);
    ctx.stroke();

    ctx.fillStyle = '#1e293b';
    ctx.beginPath();
    ctx.arc(bobX, bobY, 16, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.fillStyle = '#f8fafc';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Rotational', pivotX, pivotY - 20);
    ctx.fillStyle = '#94a3b8';
    ctx.fillText('Equivalent', pivotX, pivotY - 8);
}

function draw() {
    const width = canvas.width / window.devicePixelRatio;
    const height = canvas.height / window.devicePixelRatio;

    ctx.clearRect(0, 0, width, height);

    const marginLeft = 380;
    const marginRight = 50; // Increased slightly for the 60.0s label
    const graphWidth = width - marginLeft - marginRight;
    const graphHeight = height - 100;

    const yScale = graphHeight / 1.8;
    const yZero = height - 50 - (0.2 * yScale);
    const yTarget = yZero - (setpoint * yScale);

    const state = trajectory[currentStep] || trajectory[0];

    // --- 1. DRAW GRAPH AXES & GUIDES ---
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.moveTo(20, yZero);
    ctx.lineTo(width - marginRight + 10, yZero);
    ctx.stroke();

    ctx.strokeStyle = '#ef4444';
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.moveTo(20, yTarget);
    ctx.lineTo(width - marginRight + 10, yTarget);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = '#ef4444';
    ctx.font = '12px monospace';
    ctx.fillText('Target (x=1.0, θ=0)', marginLeft, yTarget - 8);

    // --- DYNAMIC GRAPH SCALING ---
    // Minimum view window is 10s. If simulation goes beyond 10s, scale dynamically up to 60s
    const currentT = currentStep * dt;
    const viewTime = Math.max(10.0, currentT);
    const viewSteps = viewTime / dt;
    const maxDrawStep = Math.min(totalSteps, Math.ceil(viewSteps));

    // --- 2. DRAW GRAPH TRAJECTORY ---
    // Draw the predicted gray line up to the bounds of the current view window
    ctx.beginPath();
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.5;
    for (let i = 0; i <= maxDrawStep; i++) {
        const x = marginLeft + (i / viewSteps) * graphWidth;
        const y = yZero - (trajectory[i].p * yScale);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Draw the active blue trajectory
    if (currentStep > 0) {
        ctx.beginPath();
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 3;

        for (let i = 0; i <= currentStep; i++) {
            const x = marginLeft + (i / viewSteps) * graphWidth;
            const y = yZero - (trajectory[i].p * yScale);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        const currentX = marginLeft + (currentStep / viewSteps) * graphWidth;
        const currentY = yZero - (state.p * yScale);

        ctx.fillStyle = '#10b981';
        ctx.beginPath();
        ctx.arc(currentX, currentY, 6, 0, Math.PI * 2);
        ctx.fill();
    }

    ctx.fillStyle = '#64748b';
    ctx.font = '11px sans-serif';
    ctx.fillText('Position Response', marginLeft - 10, yTarget - 25);
    ctx.fillText('0.0s', marginLeft, yZero + 20);

    // Dynamically update the right-side label based on the current scale
    ctx.textAlign = 'right';
    ctx.fillText(`${viewTime.toFixed(1)}s`, width - marginRight + 10, yZero + 20);
    ctx.textAlign = 'left'; // Reset

    // --- 3. DRAW PHYSICAL SYSTEMS (Left Side) ---
    const massY = yZero - (state.p * yScale);
    const massHeight = 24;
    const massWidth = 60;

    const linearCenterX = 70;
    const rotationalCenterX = 180;

    drawSpring(ctx, linearCenterX, yTarget, massY - massHeight/2);

    ctx.fillStyle = '#1e293b';
    ctx.fillRect(linearCenterX - massWidth/2, massY - massHeight/2, massWidth, massHeight);
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 2;
    ctx.strokeRect(linearCenterX - massWidth/2, massY - massHeight/2, massWidth, massHeight);

    ctx.fillStyle = '#f8fafc';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Linear', linearCenterX, massY - 2);
    ctx.fillStyle = '#94a3b8';
    ctx.fillText('Mass', linearCenterX, massY + 10);

    const pendulumLength = (yZero - yTarget) * 0.9;
    const errorPercent = setpoint - state.p;
    drawPendulum(ctx, rotationalCenterX, yTarget, errorPercent, pendulumLength);

    // --- 4. DRAW FORCE VECTORS (Free Body Diagram) ---
    ctx.textAlign = 'left';
    ctx.font = '12px monospace';
    ctx.fillStyle = '#38bdf8';
    ctx.fillText(`Stiffness force (throttle) = ${state.Fp.toFixed(1)}`, 20, 25);
    ctx.fillStyle = '#f43f5e';
    ctx.fillText(`Damping force (brakes) = ${state.Fd.toFixed(1)}`, 20, 45);
    ctx.fillStyle = '#10b981';
    ctx.fillText(`Net Total = ${state.Ftotal.toFixed(1)}`, 20, 65);
}

recomputeTrajectory();

let lastTime = performance.now();

function loop(/** @type {number} */ currentTime) {
    const delta = (currentTime - lastTime) / 1000;
    lastTime = currentTime;

    if (isPlaying) {
        // Apply playback speed modifier instead of strictly playing 1:1 real-time
        exactStep += (delta * playbackSpeed) / dt;
        currentStep = Math.floor(exactStep);

        if (currentStep >= totalSteps - 1) {
            currentStep = totalSteps - 1;
            exactStep = currentStep;
            isPlaying = false;
            btnPlay.textContent = 'Play';
        }
    } else {
        // Ensure float tracker stays in sync if manually scrubbed while paused
        exactStep = currentStep;
    }

    scrubber.value = String((currentStep / (totalSteps - 1)) * 1000);
    const currentTimeSec = (currentStep * dt).toFixed(2);
    timeReadout.textContent = `t = ${currentTimeSec}s / ${totalTime.toFixed(2)}s`;

    draw();
    requestAnimationFrame(loop);
}

requestAnimationFrame(loop);
