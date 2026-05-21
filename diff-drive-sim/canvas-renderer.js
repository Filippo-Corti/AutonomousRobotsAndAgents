export function createCanvasRenderer(canvas) {
    const context = canvas.getContext("2d");

    function toCanvas(point) {
        const { width, height } = canvas.getBoundingClientRect();
        const centerX = width / 2;
        const centerY = height / 2;
        const scale = Math.min(width, height) * 0.22;

        return {
            x: centerX + point.x * scale,
            y: centerY - point.y * scale,
            scale,
        };
    }

    function drawGrid(width, height, scale) {
        const spacing = scale * 0.5;
        context.strokeStyle = "rgba(171, 192, 221, 0.09)";
        context.lineWidth = 1;

        for (let x = width / 2; x <= width; x += spacing) {
            context.beginPath();
            context.moveTo(x, 0);
            context.lineTo(x, height);
            context.stroke();
        }

        for (let x = width / 2; x >= 0; x -= spacing) {
            context.beginPath();
            context.moveTo(x, 0);
            context.lineTo(x, height);
            context.stroke();
        }

        for (let y = height / 2; y <= height; y += spacing) {
            context.beginPath();
            context.moveTo(0, y);
            context.lineTo(width, y);
            context.stroke();
        }

        for (let y = height / 2; y >= 0; y -= spacing) {
            context.beginPath();
            context.moveTo(0, y);
            context.lineTo(width, y);
            context.stroke();
        }

        context.strokeStyle = "rgba(236, 242, 255, 0.24)";
        context.beginPath();
        context.moveTo(0, height / 2);
        context.lineTo(width, height / 2);
        context.stroke();

        context.beginPath();
        context.moveTo(width / 2, 0);
        context.lineTo(width / 2, height);
        context.stroke();
    }

    function drawTrajectory(history) {
        if (history.length < 2) {
            return;
        }

        context.beginPath();
        const firstPoint = toCanvas(history[0]);
        context.moveTo(firstPoint.x, firstPoint.y);

        for (let index = 1; index < history.length; index += 1) {
            const point = toCanvas(history[index]);
            context.lineTo(point.x, point.y);
        }

        context.strokeStyle = "rgba(139, 231, 143, 0.9)";
        context.lineWidth = 2.5;
        context.lineJoin = "round";
        context.lineCap = "round";
        context.stroke();

        const current = toCanvas(history[history.length - 1]);
        context.fillStyle = "rgba(139, 231, 143, 0.95)";
        context.beginPath();
        context.arc(current.x, current.y, 4, 0, Math.PI * 2);
        context.fill();
    }

    function drawRobot(pose) {
        const { x, y, scale } = toCanvas(pose);
        const bodyRadius = scale * 0.38;
        const wheelHalfLength = scale * 0.24;
        const wheelHalfWidth = scale * 0.06;

        context.save();
        context.translate(x, y);
        context.rotate(-pose.theta);

        context.fillStyle = "rgba(36, 47, 71, 0.95)";
        context.strokeStyle = "rgba(214, 228, 255, 0.7)";
        context.lineWidth = 2;
        context.beginPath();
        context.roundRect(-bodyRadius, -bodyRadius, bodyRadius * 2, bodyRadius * 2, bodyRadius * 0.35);
        context.fill();
        context.stroke();

        context.fillStyle = "rgba(88, 208, 106, 0.9)";
        context.beginPath();
        context.roundRect(bodyRadius * 0.45, -wheelHalfWidth, wheelHalfLength, wheelHalfWidth * 2, wheelHalfWidth);
        context.fill();

        context.fillStyle = "rgba(255, 160, 85, 0.9)";
        context.beginPath();
        context.roundRect(-bodyRadius - wheelHalfLength * 0.4, -wheelHalfWidth, wheelHalfLength, wheelHalfWidth * 2, wheelHalfWidth);
        context.fill();

        context.strokeStyle = "rgba(255, 255, 255, 0.95)";
        context.lineWidth = 3;
        context.beginPath();
        context.moveTo(0, 0);
        context.lineTo(scale * 0.55, 0);
        context.stroke();

        context.beginPath();
        context.moveTo(scale * 0.55, 0);
        context.lineTo(scale * 0.42, -scale * 0.08);
        context.lineTo(scale * 0.42, scale * 0.08);
        context.closePath();
        context.fillStyle = "rgba(255, 255, 255, 0.95)";
        context.fill();

        context.restore();
    }

    function resizeToParent() {
        const parent = canvas.parentElement;
        const dpr = window.devicePixelRatio || 1;
        const width = Math.max(1, parent.clientWidth);
        const height = Math.max(1, parent.clientHeight);

        canvas.width = Math.round(width * dpr);
        canvas.height = Math.round(height * dpr);
        canvas.style.width = `${width}px`;
        canvas.style.height = `${height}px`;
        context.setTransform(dpr, 0, 0, dpr, 0, 0);

        return { width, height };
    }

    function render({ pose, history }) {
        const bounds = canvas.getBoundingClientRect();
        const width = bounds.width;
        const height = bounds.height;

        context.clearRect(0, 0, width, height);

        const scale = Math.min(width, height) * 0.22;
        drawGrid(width, height, scale);
        drawTrajectory(history);
        drawRobot(pose);
    }

    return {
        resizeToParent,
        render,
    };
}