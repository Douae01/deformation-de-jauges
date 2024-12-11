const express = require('express');
const { createServer } = require('http');
const { Server } = require('socket.io');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const cors = require('cors');

const app = express();
const server = createServer(app);
const io = new Server(server);
app.use(cors());
app.use(express.json()); // Middleware to parse JSON bodies

let port;
let parser;

// Function to set up the serial port based on the communication method
const setupSerialPort = (path) => {
  if (port) {
    port.close((err) => {
      if (err) console.error('Failed to close existing serial port:', err);
    }); // Close the existing port if open
  }
  port = new SerialPort({ path, baudRate: 115200 }, (err) => {
    if (err) {
      return console.error(`Failed to open serial port at ${path}:`, err.message);
    }
    console.log(`Serial port opened at ${path}`);
  });

  parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }));

  parser.on('data', (data) => {
    console.log('Data from serial port:', data);
    io.emit('serialData', data); // Emit the data to the connected clients
  });

  port.on('error', (err) => {
    console.error('Serial port error:', err.message);
  });
};

// Allow serial port path to be passed as an argument or use default
const serialPortPath = process.argv[2] || '/dev/ttyACM0';
setupSerialPort(serialPortPath);

// Socket.IO connection handling
io.on('connection', (socket) => {
  console.log('Client connected');

  socket.on('message', (data) => {
    console.log('Message from client:', data);
    // Write data to the serial port
    port.write(data);
  });

  // Listen for disconnection
  socket.on('disconnect', () => {
    console.log('Client disconnected');
  });
});


// Graceful shutdown on server close
const cleanup = () => {
  if (port && port.isOpen) {
    port.close((err) => {
      if (err) {
        console.error('Error closing serial port:', err.message);
      } else {
        console.log('Serial port closed');
      }
    });
  }
  server.close(() => {
    console.log('HTTP server closed');
  });
};

process.on('SIGINT', cleanup);
process.on('SIGTERM', cleanup);

// Start the HTTP server
const PORT = process.env.PORT || 5000;
server.listen(PORT, () => {
  console.log(`Server listening on port ${PORT}`);
});