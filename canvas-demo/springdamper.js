// @ts-check

// ---------------------------------------------------------
// 1. DOM Elements
// ---------------------------------------------------------
const canvasElement = /** @type {HTMLCanvasElement} */ (document.getElementById('simCanvas'));
const DOM = {
    canvas:      /** @type {HTMLCanvasElement} */ canvasElement,
    ctx:         /** @type {CanvasRenderingContext2D} */ (canvasElement.getContext('2d')),
    sliderKp:    /** @type {HTMLInputElement} */  (document.getElementById('sliderKp')),
    sliderKd:    /** @type {HTMLInputElement} */  (document.getElementById('sliderKd')),
    scrubber:    /** @type {HTMLInputElement} */  (document.getElementById('scrubber')),
    valKp:       document.getElementById('valKp'),
    valKd:       document.getElementById('valKd'),
    btnPlay:     document.getElementById('btnPlay'),
    btnReset:    document.getElementById('btnReset'),
    timeReadout: document.getElementById('timeReadout'),
    toggleVel:   document.getElementById('toggleVel'),
    toggleFp:    document.getElementById('toggleFp'),
    toggleFd:    document.getElementById('toggleFd'),
    toggleFtotal:document.getElementById('toggleFtotal')
};

// ---------------------------------------------------------
// 2. Simulation State & Constants
// ---------------------------------------------------------
const Sim = {
    m: 1.0, dt: 0.01, totalTime: 60.0, setpoint: 1.0, playbackSpeed: 0.5,
    Kp: parseFloat(DOM.sliderKp.value),
    Kd: parseFloat(DOM.sliderKd.value),
    get totalSteps() { return Math.floor(this.totalTime / this.dt); },
    isPlaying: true, currentStep: 0, exactStep: 0,
    trajectory: /** @type {Array<any>} */ ([])
};
Sim.trajectory = new Array(Sim.totalSteps);

const PlotVisibility = {
    v: false,
    Fp: false,
    Fd: false,
    Ftotal: false
};

// ---------------------------------------------------------
// 3. Physics & Math (RK4 Integration)
// ---------------------------------------------------------
function recomputeTrajectory() {
    let pos = 0.0, vel = 0.0;

    const getDerivatives = (/** @type {number} */ p, /** @type {number} */ v) => ({
        dPos: v,
        dVel: (Sim.Kp * (Sim.setpoint - p) - Sim.Kd * v) / Sim.m
    });

    for (let i = 0; i < Sim.totalSteps; i++) {
        const Fp = Sim.Kp * (Sim.setpoint - pos);
        const Fd = -Sim.Kd * vel;

        Sim.trajectory[i] = { p: pos, v: vel, Fp, Fd, Ftotal: Fp + Fd };

        const k1 = getDerivatives(pos, vel);
        const k2 = getDerivatives(pos + 0.5 * Sim.dt * k1.dPos, vel + 0.5 * Sim.dt * k1.dVel);
        const k3 = getDerivatives(pos + 0.5 * Sim.dt * k2.dPos, vel + 0.5 * Sim.dt * k2.dVel);
        const k4 = getDerivatives(pos + Sim.dt * k3.dPos, vel + Sim.dt * k3.dVel);

        pos += (Sim.dt / 6) * (k1.dPos + 2 * k2.dPos + 2 * k3.dPos + k4.dPos);
        vel += (Sim.dt / 6) * (k1.dVel + 2 * k2.dVel + 2 * k3.dVel + k4.dVel);
    }
}

// ---------------------------------------------------------
// 4. UI Listeners & Canvas Sizing
// ---------------------------------------------------------
function resizeCanvas() {
    const rect = DOM.canvas.parentElement.getBoundingClientRect();
    DOM.canvas.width = rect.width * window.devicePixelRatio;
    DOM.canvas.height = rect.height * window.devicePixelRatio;
    DOM.ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
}
window.addEventListener('resize', resizeCanvas);

DOM.sliderKp.addEventListener('input', e => {
    Sim.Kp = parseFloat(/** @type {HTMLInputElement} */(e.target).value);
    DOM.valKp.textContent = Sim.Kp.toFixed(1);
    recomputeTrajectory();
});

DOM.sliderKd.addEventListener('input', e => {
    Sim.Kd = parseFloat(/** @type {HTMLInputElement} */(e.target).value);
    DOM.valKd.textContent = Sim.Kd.toFixed(1);
    recomputeTrajectory();
});

DOM.scrubber.addEventListener('input', e => {
    const progress = parseFloat(/** @type {HTMLInputElement} */(e.target).value) / 1000;
    Sim.exactStep = Sim.currentStep = Math.floor(progress * (Sim.totalSteps - 1));
    Sim.isPlaying = false;
    DOM.btnPlay.textContent = 'Play';
});

DOM.btnPlay.addEventListener('click', () => {
    Sim.isPlaying = !Sim.isPlaying;
    DOM.btnPlay.textContent = Sim.isPlaying ? 'Pause' : 'Play';
    if (Sim.isPlaying && Sim.currentStep >= Sim.totalSteps - 1) {
        Sim.currentStep = Sim.exactStep = 0;
    }
});

DOM.btnReset.addEventListener('click', () => {
    Sim.currentStep = Sim.exactStep = 0;
    Sim.isPlaying = true;
    DOM.btnPlay.textContent = 'Pause';
});

/**
 * @param {HTMLElement | null} element
 * @param {'v' | 'Fp' | 'Fd' | 'Ftotal'} key
 * @param {string} color
 */
function setupEquationToggle(element, key, color) {
    if (!element) return;
    element.addEventListener('click', () => {
        PlotVisibility[key] = !PlotVisibility[key];
        if (PlotVisibility[key]) {
            element.style.outline = `2px solid ${color}`;
            element.style.backgroundColor = `${color}22`; // Add a subtle highlight tint
        } else {
            element.style.outline = 'none';
            element.style.backgroundColor = '';
        }
        if (!Sim.isPlaying) draw();
    });
}

setupEquationToggle(DOM.toggleVel, 'v', '#ffdf7c');
setupEquationToggle(DOM.toggleFp, 'Fp', '#38bdf8');
setupEquationToggle(DOM.toggleFd, 'Fd', '#f43f5e');
setupEquationToggle(DOM.toggleFtotal, 'Ftotal', '#10b981');

// ---------------------------------------------------------
// 5. Drawing Routines
// ---------------------------------------------------------

/**
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} xLeft
 * @param {number} xRight
 * @param {number} y
 */
function drawHorizontalSpring(ctx, xLeft, xRight, y) {
    const coils = 12, h = 12, pad = 8;
    const startX = xLeft + pad, endX = xRight - pad;
    const dx = (endX - startX) / coils;

    ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 2.5; ctx.lineJoin = 'round';
    ctx.beginPath(); ctx.moveTo(xLeft, y); ctx.lineTo(startX, y);

    for (let i = 0; i < coils; i++) {
        ctx.lineTo(startX + i * dx + dx / 2, y + (i % 2 === 0 ? -h : h));
    }

    ctx.lineTo(endX, y); ctx.lineTo(xRight, y); ctx.stroke();
}

/**
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} pivotX
 * @param {number} pivotY
 * @param {number} errorPercent
 * @param {number} length
 */
function drawPendulum(ctx, pivotX, pivotY, errorPercent, length) {
    const angle = -errorPercent * (Math.PI / 2.5);

    // Draw pivot dashed line
    ctx.strokeStyle = '#ef4444'; ctx.setLineDash([4, 4]); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(pivotX, pivotY); ctx.lineTo(pivotX, pivotY + length + 25); ctx.stroke();

    // Draw pivot circle
    ctx.setLineDash([]); ctx.fillStyle = '#94a3b8';
    ctx.beginPath(); ctx.arc(pivotX, pivotY, 5, 0, Math.PI * 2); ctx.fill();

    // Draw arm and bob
    const bobX = pivotX + length * Math.sin(angle), bobY = pivotY + length * Math.cos(angle);
    ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(pivotX, pivotY); ctx.lineTo(bobX, bobY); ctx.stroke();

    ctx.fillStyle = '#1e293b'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(bobX, bobY, 16, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

    // Labels
    ctx.fillStyle = '#f8fafc'; ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
    ctx.fillText('Rotational', pivotX, pivotY + 20);
    ctx.fillStyle = '#94a3b8'; ctx.fillText('Equivalent', pivotX, pivotY + 30);
}

function draw() {
    const { ctx, canvas } = DOM;
    const width = canvas.width / window.devicePixelRatio;
    const height = canvas.height / window.devicePixelRatio;
    ctx.clearRect(0, 0, width, height);

    const marginLeft = 320, marginRight = 50;
    const graphWidth = width - marginLeft - marginRight;
    const yScale = (height - 100) / 1.8;
    const yZero = height - 50 - (0.2 * yScale);
    const yTarget = yZero - (Sim.setpoint * yScale);
    const state = Sim.trajectory[Sim.currentStep] || Sim.trajectory[0];

    // --- A. GRAPH AXES ---
    ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(20, yZero); ctx.lineTo(width - marginRight + 10, yZero); ctx.stroke();

    ctx.strokeStyle = '#ef4444'; ctx.setLineDash([6, 6]);
    ctx.beginPath(); ctx.moveTo(20, yTarget); ctx.lineTo(width - marginRight + 10, yTarget); ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = '#ef4444'; ctx.font = '12px monospace'; ctx.textAlign = 'left';
    ctx.fillText('Target (x=1.0, θ=0)', marginLeft, yTarget - 8);

    // --- B. TRAJECTORY LINE ---
    const currentT = Sim.currentStep * Sim.dt;

    // Define bounds (in seconds)
    const minWindow = 1.0;  // Initial viewport span (tight & zoomed in)
    const maxWindow = 60.0; // Total trajectory span

    // Gradually expand the view window using a soft curve
    const visibleWindow = Math.min(
        maxWindow,
        Math.max(minWindow, Math.pow(currentT, 0.85) + minWindow)
    );

    const viewSteps = visibleWindow / Sim.dt;
    const maxDrawStep = Math.min(Sim.totalSteps, Math.ceil(viewSteps));

    /**
     * Helper to plot any trajectory metric
     * @param {'p'|'v'|'Fp'|'Fd'|'Ftotal'} metric
     * @param {string} baseColor
     * @param {string} activeColor
     * @param {number} lineWidth
     * @param {number} scaleMult
     * @param {boolean} isDashed
     */
    const drawGraphLine = (metric, baseColor, activeColor, lineWidth, scaleMult = 1.0, isDashed = false) => {
        if (isDashed) ctx.setLineDash([5, 5]);

        // Faint future/full path
        ctx.beginPath(); ctx.strokeStyle = baseColor; ctx.lineWidth = lineWidth * 0.5;
        for (let i = 0; i <= maxDrawStep; i++) {
            const x = marginLeft + (i / viewSteps) * graphWidth;
            const y = yZero - (Sim.trajectory[i][metric] * scaleMult * yScale);
            i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Bright active path
        if (Sim.currentStep > 0) {
            ctx.beginPath(); ctx.strokeStyle = activeColor; ctx.lineWidth = lineWidth;
            for (let i = 0; i <= Sim.currentStep; i++) {
                const x = marginLeft + (i / viewSteps) * graphWidth;
                const y = yZero - (Sim.trajectory[i][metric] * scaleMult * yScale);
                i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
            }
            ctx.stroke();

            // Lead point indicator
            const cx = marginLeft + (Sim.currentStep / viewSteps) * graphWidth;
            const cy = yZero - (Sim.trajectory[Sim.currentStep][metric] * scaleMult * yScale);
            ctx.fillStyle = activeColor;
            ctx.beginPath(); ctx.arc(cx, cy, metric === 'p' ? 6 : 4, 0, Math.PI * 2); ctx.fill();
        }

        ctx.setLineDash([]); // Reset dash for the next drawing operations
    };

    // 1. Draw toggled metrics (Scaled down by 10% to fit neatly on screen)
    const forceScale = 0.1;

    if (PlotVisibility.v)      drawGraphLine('v',      '#0f172a', '#ffdf7c', 2, 1.5*forceScale, true);
    if (PlotVisibility.Fp)     drawGraphLine('Fp',     '#0f172a', '#38bdf8', 2, 0.1*forceScale);
    if (PlotVisibility.Fd)     drawGraphLine('Fd',     '#0f172a', '#f43f5e', 2, 0.5*forceScale);
    if (PlotVisibility.Ftotal) drawGraphLine('Ftotal', '#0f172a', '#10b981', 2, 0.5*forceScale);

    // 2. Draw Position (Always drawn, bright white/gray)
    drawGraphLine('p', '#334155', '#f8fafc', 3, 1.0);

    // Graph Text Labels
    ctx.fillStyle = '#64748b'; ctx.font = '11px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('Position Response', marginLeft - 10, yTarget - 25);
    ctx.fillText('0.0s', marginLeft, yZero + 20);
    ctx.textAlign = 'right';
    ctx.fillText(`${(viewSteps * Sim.dt).toFixed(1)}s`, width - marginRight + 10, yZero + 20);

    // --- C. PHYSICAL SYSTEMS ---
    const sysCX = 180, linY = yTarget - 30, penY = yTarget;

    // Target Line
    ctx.strokeStyle = '#ef4444'; ctx.setLineDash([3, 3]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(sysCX, linY - 50); ctx.lineTo(sysCX, penY + 80); ctx.stroke();
    ctx.setLineDash([]);

    // Linear System
    const mX = sysCX - (Sim.setpoint - state.p) * 60;
    ctx.strokeStyle = '#64748b'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(30, linY - 25); ctx.lineTo(30, linY + 25); ctx.stroke();
    ctx.lineWidth = 1;
    for (let hY = linY - 20; hY <= linY + 20; hY += 8) {
        ctx.beginPath(); ctx.moveTo(22, hY + 8); ctx.lineTo(30, hY); ctx.stroke();
    }

    drawHorizontalSpring(ctx, 30, mX - 32.5, linY);
    ctx.fillStyle = '#1e293b'; ctx.fillRect(mX - 32.5, linY - 12, 65, 24);
    ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 2; ctx.strokeRect(mX - 32.5, linY - 12, 65, 24);
    ctx.fillStyle = '#f8fafc'; ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
    ctx.fillText('1D', mX, linY + 4);

    // Pendulum System
    drawPendulum(ctx, sysCX, penY, Sim.setpoint - state.p, 170);

    // --- D. FORCE READOUTS ---
    ctx.textAlign = 'left'; ctx.font = '12px monospace';
    ctx.fillStyle = '#38bdf8'; ctx.fillText(`Stiffness force = ${state.Fp.toFixed(1)}`, 20, 25);
    ctx.fillStyle = '#f43f5e'; ctx.fillText(`Damping force   = ${state.Fd.toFixed(1)}`, 20, 45);
    ctx.fillStyle = '#10b981'; ctx.fillText(`Net Total       = ${state.Ftotal.toFixed(1)}`, 20, 65);
}

// ---------------------------------------------------------
// 6. Main Loop
// ---------------------------------------------------------
let lastTime = performance.now();

function loop(/** @type {number} */ currentTime) {
    const delta = (currentTime - lastTime) / 1000;
    lastTime = currentTime;

    if (Sim.isPlaying) {
        Sim.exactStep += (delta * Sim.playbackSpeed) / Sim.dt;
        Sim.currentStep = Math.floor(Sim.exactStep);

        if (Sim.currentStep >= Sim.totalSteps - 1) {
            Sim.exactStep = Sim.currentStep = Sim.totalSteps - 1;
            Sim.isPlaying = false;
            DOM.btnPlay.textContent = 'Play';
        }
    } else {
        Sim.exactStep = Sim.currentStep;
    }

    DOM.scrubber.value = String((Sim.currentStep / (Sim.totalSteps - 1)) * 1000);
    DOM.timeReadout.textContent = `t = ${(Sim.currentStep * Sim.dt).toFixed(2)}s / ${Sim.totalTime.toFixed(2)}s`;

    draw();
    requestAnimationFrame(loop);
}

// Init
resizeCanvas();
recomputeTrajectory();
requestAnimationFrame(loop);
