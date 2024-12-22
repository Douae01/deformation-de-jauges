import React, { useEffect, useRef } from "react";
import Chart from "chart.js/auto";

const LineChart = ({ data, label, unit, color }) => {
  const chartRef = useRef(null);
  const chartInstance = useRef(null);

  useEffect(() => {
    if (chartRef.current && data.length > 0) {
      if (chartInstance.current) {
        chartInstance.current.data.labels = data.map((_, index) => index);
        chartInstance.current.data.datasets[0].data = data;
        chartInstance.current.options.scales.y.title.text = `${label} (${unit})`;
        chartInstance.current.options.scales.y.min = 0;
        chartInstance.current.options.scales.y.max = unit === "V" ? 5 : 5000;
        chartInstance.current.update();
      } else {
        const ctx = chartRef.current.getContext("2d");
        chartInstance.current = new Chart(ctx, {
          type: "line",
          data: {
            labels: data.map((_, index) => index),
            datasets: [
              {
                label: label,
                data: data,
                borderColor: color,
                backgroundColor: color,
                borderWidth: 1,
              },
            ],
          },
          options: {
            scales: {
              y: {
                min: unit === "V" ? -5 : 0,
                max: unit === "V" ? 40 : 100,
                display: true,
                title: {
                  display: true,
                  text: `${label} (${unit})`,
                },
              },
            },
          },
        });
      }
    }
  }, [data, unit]);

  return <canvas ref={chartRef}></canvas>;
};

export default LineChart;