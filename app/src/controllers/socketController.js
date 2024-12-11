import socket from "../models/socketInit";

// Initialize socket event listeners
const initializeSocket = (setCurrentAngle, setHasConnection) => {
  const fetchData = (data) => {
    const [, anglePart] = data.split(",");
    const angle = parseInt(anglePart.split(":")[1].trim(), 10);
    setCurrentAngle(angle);
    setHasConnection(true);
  };

  const handleDisconnect = () => {
    setHasConnection(false);
  };

  // Listen for serialData and disconnect events
  socket.on("serialData", fetchData);
  socket.on("disconnect", handleDisconnect);
};

// Cleanup socket event listeners
const cleanupSocket = () => {
  socket.off("serialData");
  socket.off("disconnect");
};

export default {
  initializeSocket,
  cleanupSocket,
};
