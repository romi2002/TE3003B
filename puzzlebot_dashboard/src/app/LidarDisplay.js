import {useEffect, useRef} from "react";

export default function LidarDisplay({distances, maxDistance, width, height}) {
    const canvasRef = useRef(null);
    useEffect(() => {
        const canvas = canvasRef.current;
        const ctx = canvas.getContext('2d');
        const width = canvas.width;
        const height = canvas.height;
        const centerX = width / 2;
        const centerY = height / 2;
        console.log(distances)

        // Clear the canvas
        ctx.clearRect(0, 0, width, height);

        // Draw the circle
        ctx.beginPath();
        ctx.arc(centerX, centerY, maxDistance, 0, 2 * Math.PI);
        ctx.strokeStyle = '#000';
        ctx.stroke();

        distances.forEach((distance, index) => {
            distance = Math.min(Math.max(distance, 0), maxDistance);
            const angle = -Math.PI + (2 * Math.PI * index) / distances.length;
            const x = centerX + distance * Math.cos(angle);
            const y = centerY + distance * Math.sin(angle);

            // Draw line from center to point
            ctx.beginPath();
            ctx.moveTo(centerX, centerY);
            ctx.lineTo(x, y);
            ctx.strokeStyle = 'blue';
            ctx.stroke();
        });

        // Draw center circle.
        ctx.beginPath()
        ctx.arc(centerX, centerY, 10, 0, 2 * Math.PI);
        ctx.strokeStyle = '#ff0000'
        ctx.fillStyle = "red";
        ctx.fill();
        ctx.stroke();
    }, [distances]);

    return (
        <div>
            <canvas ref={canvasRef} width={width} height={height}/>
        </div>
    )
}
