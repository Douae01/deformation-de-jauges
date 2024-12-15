import React, { useState } from "react";
import Logo from "../assets/icons/logo.png";
import { Link } from "react-router-dom";
import { AiOutlineMenu, AiOutlineClose } from "react-icons/ai"; // Menu icons

function LeftSidebar({ children, activeChild }) {
  const [isOpen, setIsOpen] = useState(false); /* Sidebar visibility state */

  /* 
   * renderChildren: Maps over `children` to render each navigation item as a list element.
   * Highlights the item if its name matches `activeChild`.
   */
  const renderChildren = () => {
    return React.Children.map(children, (child) => {
      const isActive = child.props.name === activeChild;
      
      return (
        <li key={child.props.name}>
          <Link
            to={child.props.href}
            className={`flex items-center  gap-x-2 text-grey-500 p-2 rounded-lg hover:bg-gray-50 active:bg-gray-100 duration-150 ${
              isActive ? "bg-[#a3d7e6] text-white" : ""
            }`}
          >
            <div className="text-gray-500">{child.props.icon}</div>
            {child.props.name}
          </Link>
        </li>
      );
    });
  };

  return (
    <>
      {/* Menu button for small screens */}
      <button 
        className="p-2 rounded-md bg-gray-200 m-2 xl:hidden" /* Only show on small screens */
        onClick={() => setIsOpen(true)} /* Open sidebar */
      >
        <AiOutlineMenu size={24} />
      </button>
      
      {/* Sidebar */}
      <nav
        className={`fixed top-0 left-0 h-full bg-white border-r transition-transform transform xl:translate-x-0 z-50 ${
          isOpen ? 'translate-x-0' : '-translate-x-full'
        }`}
        style={{ width: '240px' }}
      >
        <div className="flex flex-col h-full justify-between">
          {/* Logo and Close button */}
          <div className="h-20 flex items-center justify-between px-4">
            <Link to="/home" className="flex-none">
              <img src={Logo} width={160} className="mx-auto mt-4 mb-2" alt="Logo" />
            </Link>
            <button 
              className="xl:hidden p-2" 
              onClick={() => setIsOpen(false)} /* Close sidebar */
            >
              <AiOutlineClose size={24} />
            </button>
          </div>

          {/* Sidebar content */}
          <div className="flex-1 flex flex-col overflow-auto mt-2">
            <ul className="px-4 text-sm font-medium">
              {renderChildren()}
            </ul>
          </div>
        </div>
      </nav>
    </>
  );
}

export default LeftSidebar;
