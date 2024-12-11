const addRecordedStep = (action, value, recordedSteps, setRecordedSteps, recordStartTime, setRecordStartTime) => {
    const currentTime = Date.now();
    const duration = recordStartTime ? currentTime - recordStartTime : 0;

    setRecordedSteps((prevSteps) => {
        const updatedSteps = [...prevSteps];
        const latestEntry = updatedSteps[updatedSteps.length - 1];

        if (latestEntry) {
            const lastAction = latestEntry.actions[latestEntry.actions.length - 1];
            if (!lastAction || lastAction.action !== action || lastAction.speed !== value) {
                latestEntry.actions.push({
                    action,
                    speed: value,
                    duration,
                });
            }
        }

        return updatedSteps;
    });

    setRecordStartTime(currentTime);
};

const toggleRecording = (isRecording, target, positionActuelle, recordedSteps, setRecordedSteps, setRecordStartTime) => {
    if (!isRecording) {
        setRecordedSteps((prevSteps) => [
            ...prevSteps,
            {
                target,
                positionActuelle,
                actions: [],
            },
        ]);
        setRecordStartTime(Date.now());
    } else {
        console.log("Recorded steps:", recordedSteps);
        saveStepsToFile(recordedSteps);
        setRecordStartTime(null);
    }
};

const saveStepsToFile = (steps) => {
    const stepsJson = JSON.stringify(steps, null, 2);
    const blob = new Blob([stepsJson], { type: "application/json" });
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = "recorded_steps.json";
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
};

export default {
    addRecordedStep,
    toggleRecording,
};
