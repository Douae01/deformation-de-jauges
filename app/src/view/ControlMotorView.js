import React, { useState, useEffect, useRef } from "react";
import socket from "../models/socketInit";
import LineChart from "../components/LineChart";
import Balance from "../assets/balance.png"

const ControlMotorView = () => {
    const [realTimeData, setRealTimeData] = useState([]);
    const [currentAngle, setCurrentAngle] = useState(0);
    const [hasConnection, setHasConnection] = useState(true);
    const [mode, setMode] = useState("manual"); // "manual" or "automatic"
    const [masse, setMasse] = useState(0);
    const [deformation, setDeformation] = useState(0);
    const [force, setForce] = useState(0);
    const [distance, setDistance] = useState(0);
    const canvasRef = useRef(null);
    const deformationCanvasRef = useRef(null);
    
    useEffect(() => {

        const fetchData = (data) => {
            try {
                // Extraire les données du message reçu : voltage
                if (data.startsWith("V:")) {
                    const voltValueString = data.substring(2);
                    const voltValue = parseFloat(voltValueString);
                    console.log("Tension extraite :", voltValue);
                    setRealTimeData((prevData) => [...prevData, voltValue]);
                }
                else if (data.startsWith("D:")) {
                    const deformationValueString = data.substring(2);
                    const deformationValue = parseFloat(deformationValueString);
                    console.log("Deformation extraite :", deformationValue);
                    setDeformation(deformationValue);
                }
                else if (data.startsWith("M:")) {
                    const masseValueString = data.substring(2);
                    const masseValue = parseFloat(masseValueString);
                    const masse= masseValue;
                    console.log("Masse extraite :", masse);
                    setMasse(masse);
                }
                else if (data.startsWith("F:")) {
                    const forceValueString = data.substring(2);
                    const force = parseFloat(forceValueString);
                    console.log("Effort extrait :", force);
                    setForce(force);
                }
                else {
                    // Extraire les données du message reçu : "Mode: X, Angle: Y"
                    const [, anglePart] = data.split(",");
                    const angle = parseInt(data);

                    // Mettre à jour l'angle actuel et enregistrer dans la liste
                    setCurrentAngle(angle);
                    setAngles((prevAngles) => [...prevAngles, angle]);
                }


                // Si la connexion était perdue, la rétablir
                if (!hasConnection) {
                    setHasConnection(true);
                }
            } catch (error) {
                console.error("Erreur lors de l'analyse des données reçues :", error);
            }
        };

        const handleDisconnect = () => {
            setHasConnection(false);
        };
        socket.on("serialData", fetchData);
        socket.on("disconnect", handleDisconnect);
    }, [hasConnection,realTimeData]);
    
    const handleAngleChange = (event) => {
        const newAngle = event.target.value;
        setCurrentAngle(newAngle);
        if (mode === "manual") {
            socket.emit("message", `1,${newAngle}e`);  // Mode manuel, servo activé, angle
        }
    };
    
    const handleModeChange = (event) => {
        const newMode = event.target.value;
        setMode(newMode);
        if (newMode === "automatic") {
            socket.emit("message", `2,1e`);  // Mode automatique, servo activé
        }else{
            socket.emit("message", `1,90e`);
        }
    };


    useEffect(() => {
        const canvas = canvasRef.current;
        const ctx = canvas.getContext("2d");

        const drawRadar = () => {
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            const centerX = canvas.width / 2;
            const centerY = canvas.height - 8; // Position du centre (8px du bas)
            const radius = canvas.height - 40; // Rayon du radar

            // Dessiner le fond
            ctx.fillStyle = "black";
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            // Dessiner les cercles concentriques
            ctx.strokeStyle = "green";
            ctx.lineWidth = 1;
            for (let i = 1; i <= 4; i++) {
                ctx.beginPath();
                ctx.arc(centerX, centerY, (radius / 4) * i, Math.PI, 2 * Math.PI);
                ctx.stroke();
            }

            // Dessiner les lignes radiales et les degrés
            for (let angle = 0; angle <= 180; angle += 15) {
                const rad = (Math.PI / 180) * angle;
                const x = centerX + Math.cos(rad) * radius;
                const y = centerY - Math.sin(rad) * radius;

                // Ligne radiale
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.lineTo(x, y);
                ctx.stroke();

                // Ajouter les étiquettes de degrés
                const labelX = centerX + Math.cos(rad) * (radius + 20);
                const labelY = centerY - Math.sin(rad) * (radius + 20);
                ctx.fillStyle = "green";
                ctx.font = "10px Arial";
                ctx.fillText(`${angle}°`, labelX - 3, labelY);
            }
        };

        const drawArrow = (angle) => {
            const rad = (Math.PI / 180) * angle;
            const x = canvas.width / 2 + Math.cos(rad) * (canvas.height - 40);
            const y = canvas.height - 8 - Math.sin(rad) * (canvas.height - 40);

            ctx.strokeStyle = "red";
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(canvas.width / 2, canvas.height - 8);
            ctx.lineTo(x, y);
            ctx.stroke();
        };

        drawRadar();
        drawArrow(currentAngle);
    }, [currentAngle]);

    useEffect(() => {
        const canvas = deformationCanvasRef.current;
        const ctx = canvas.getContext("2d");
    
        const drawLame = () => {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
    
            const startX = 50; // Fixed end of the beam (left side)
            const startY = canvas.height / 2; // Middle of the canvas height
            const endX = canvas.width - 50; // Free end of the beam (right side)
            const deformationEffect = deformation ? deformation * 1000 : 0; // Vertical deformation
    
            const beamThickness = 20; // Thickness of the beam for 3D effect
    
            // Top face of the beam (blue line for deformation)
            ctx.strokeStyle = "blue";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(startX, startY); // Fixed end
            ctx.lineTo(endX, startY + deformationEffect); // Free end with deformation
            ctx.stroke();
    
            // Bottom face of the beam (offset by thickness)
            ctx.strokeStyle = "darkblue";
            ctx.beginPath();
            ctx.moveTo(startX, startY + beamThickness); // Fixed end
            ctx.lineTo(endX, startY + deformationEffect + beamThickness); // Free end
            ctx.stroke();
    
            // Left side of the beam (fixed)
            ctx.fillStyle = "blue";
            ctx.beginPath();
            ctx.moveTo(startX, startY); // Top-left corner
            ctx.lineTo(startX, startY + beamThickness); // Bottom-left corner
            ctx.lineTo(startX + 10, startY + beamThickness); // Bottom-left depth
            ctx.lineTo(startX + 10, startY); // Top-left depth
            ctx.closePath();
            ctx.fill();
    
            // Top surface (3D effect)
            ctx.fillStyle = "lightblue";
            ctx.beginPath();
            ctx.moveTo(startX, startY); // Fixed top-left
            ctx.lineTo(endX, startY + deformationEffect); // Deformed top-right
            ctx.lineTo(endX + 10, startY + deformationEffect); // Depth top-right
            ctx.lineTo(startX + 10, startY); // Depth top-left
            ctx.closePath();
            ctx.fill();
    
            // Side face (free end deformation)
            ctx.fillStyle = "darkblue";
            ctx.beginPath();
            ctx.moveTo(endX, startY + deformationEffect); // Top-right
            ctx.lineTo(endX, startY + deformationEffect + beamThickness); // Bottom-right
            ctx.lineTo(endX + 10, startY + deformationEffect + beamThickness); // Depth bottom-right
            ctx.lineTo(endX + 10, startY + deformationEffect); // Depth top-right
            ctx.closePath();
            ctx.fill();
        };
    
        drawLame();
    }, [deformation]);
       

    return (
        <div className="grid grid-cols-2 grid-rows-2 min-h-screen">
            {/* Section en haut à gauche */}
            <div className="flex flex-col items-center justify-center border border-gray-300">
                {/* Contrôles en haut */}
                <div className="w-full max-w-5xl border-b-2 border-gray-300 p-5 flex flex-col items-center bg-white shadow-md">
                    <div id="currentAngle" className="text-xl font-semibold mb-2 uppercase text-shadow-10">
                        Angle Courant: {currentAngle}
                    </div>

                    <div className="mb-1">
                        <label htmlFor="modeSelect" className="mr-2 text-lg font-semibold">Mode:</label>
                        <select
                            id="modeSelect"
                            value={mode}
                            onChange={handleModeChange}
                            className="border p-2 rounded"
                        >
                            <option value="manual">Manuel</option>
                            <option value="automatic">Automatique</option>
                        </select>
                    </div>

                    {mode === "manual" && (
                        <div className="flex flex-col items-center">
                            <label htmlFor="angleRange" className="mb-1 text-lg">Angle:</label>
                            <input
                                type="range"
                                id="angleRange"
                                className="w-full max-w-md mb-2"
                                min="0"
                                max="180"
                                value={currentAngle}
                                step="10"
                                onChange={handleAngleChange}
                            />
                            <span id="angleValue" className="text-lg font-semibold">{currentAngle}</span>
                        </div>
                    )}
                </div>

                {/* Zone radar */}
                <div className="w-full flex justify-center mt-2">
                    <canvas ref={canvasRef} width="776" height="400" className="border border-gray-300 shadow-lg mb-0"></canvas>
                </div>
            </div>
        
            {/* Section en haut à droite */}
            <div className="border border-gray-300 ml-2">
                <div className="mt-0 p-0 ml-4 mt-4">
                    <p className="text-xl font-semibold mb-1">Tension au fil du temps</p>
                    <div className="w-full h-[450px]">
                        <LineChart label="Tension" data={realTimeData} unit="V" color="rgba(255, 99, 132, 1)"/>
                    </div>
                </div>
            </div>

            {/* Section en bas à gauche */}
            <div className="border border-gray-300 p-5 flex-col mt-2 h-80">
                <p className="text-xl font-semibold mb-1  m-auto">Informations nécessaires</p>
                <div className="border border-gray-300 p-5 flex flex-col items-center mt-2">
                    {/* Ajouter l'image de balance */}
                    <img src={Balance} alt="Balance" className="w-16 h-16 mb-4" />

                    {/* Informations nécessaires */}
                    <div className="text-center">
                        <div className="text-lg">
                            {/* Masse et Force */}
                            <p>Masse: {masse !== null ? masse : "Non disponible"}</p>
                            <p>Force: {force !== null ? force : "Non disponible"}</p>
                        </div>
                    </div>
                </div>
            </div>

            {/* Section en bas à droite */}
            <div className="border border-gray-300 p-5 flex mt-2 ml-2 h-80">
                <div className="m-auto">
                    <p className="text-xl font-semibold mb-5 items-center">Représentation de la lame</p>
                    <canvas ref={deformationCanvasRef} width="400" height="200" className="border border-gray-300 shadow-lg"></canvas>
                    <div className="text-center">
                        <div className="text-lg">
                            {/* Déformation */}
                            <p>Déformation: {deformation !== null ? deformation : "Non disponible"}</p>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
        
};
export default ControlMotorView;
