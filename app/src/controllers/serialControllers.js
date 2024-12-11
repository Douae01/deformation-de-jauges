const { addData } = require('../views/ui');
const { connectAndSendMessage } = require('./serialInit');

let currentAngle = 0;

async function sendMessage(message) {
    try {
        const port = await connectAndSendMessage();
        if (port) {
            port.write(`${message},${currentAngle}e`, (err) => {
                if (err) {
                    console.error('Error on write:', err.message);
                }
            });
        } else {
            console.error('Serial port is not connected.');
        }
    } catch (error) {
        console.error('Error sending message:', error);
    }
}

function setCurrentAngle(angle) {
    currentAngle = angle;
}

module.exports = { sendMessage, setCurrentAngle };
