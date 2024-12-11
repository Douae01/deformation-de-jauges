import React, { useState } from "react";
import { LayoutDashboard, Sun, SnowflakeIcon, Palette, JoystickIcon, LineChartIcon, MapIcon } from "lucide-react"; 
import Sidebar, { SidebarItem } from "./components/Sidebar.js"; 
import DashboardView from "./view/DashboardView.js"; 
import ControlMotorView from "./view/ControlMotorView.js"; 

function App() {
  const [activeComponent, setActiveComponent] = useState("Dashboard");

  const renderComponent = () => {
    switch (activeComponent) {
      case "ControlMotor":
        return <ControlMotorView />;
      case "Dashboard":
      default:
        return <DashboardView />;
    }
  };

  const changeActive = (text) => {
    setActiveComponent(text);
  }

  return (
    <>
      <div className="flex">
        <Sidebar>
          <SidebarItem icon={<LayoutDashboard size={20} />} text="Dashboard" active={activeComponent === "Dashboard"} onClick={() => changeActive("Dashboard")} />
          <SidebarItem icon={<JoystickIcon size={20} />} text="Control Motor" active={activeComponent === "ControlMotor"} onClick={() => changeActive("ControlMotor")} />
        </Sidebar>
        <div className="w-5/6 p-5">
          {renderComponent()}
        </div>
      </div>
    </>
  );
}

export default App;
