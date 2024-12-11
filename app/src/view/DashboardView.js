import React from 'react';
import logo from "../assets/enib.png"

const DashboardView = () => {
  return (
    <div className="flex flex-col items-center justify-center w-full min-h-screen">
      <div className="relative w-full bg-white rounded-lg shadow-lg p-10 h-fit-content">
        <div className="absolute top-4 left-4">
          <img src={logo} alt="Logo" className="h-16 w-18" />
        </div>
        <div className="flex flex-col items-center">
          <h1 className="text-4xl font-bold mb-6">Control Dashboard View</h1>
          <h2 className="text-2xl mb-8 text-center">
            This app is conceived by 4th year students at ENIB: 
              CHOUBRI Douae, MAMBOU Duval, BOUGHANMI Roua and LEMAIRE Rodrigue
          </h2>
          <div className="text-left w-full">
            <h1 className="text-3xl font-semibold mb-4">App Description:</h1>
            <p className="text-lg leading-relaxed mb-6">
              App Description
            </p>
          </div>
          <h3 className="text-center text-2xl font-semibold text-green-600">ENJOY OUR APP</h3>
        </div>
      </div>
    </div>
  );
};

export default DashboardView;
