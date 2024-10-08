// Navbar.tsx
import { PlayIcon, StopIcon, SunIcon, MoonIcon } from "@heroicons/react/24/outline";
import {CodeBracketIcon,CommandLineIcon} from '@heroicons/react/24/outline'
import React, { useContext } from "react";
import { Mycontext } from './CodeEditor';

interface NavbarProps {
  onRun: () => void;
  onStop: () => void;
}

const Navbar: React.FC<NavbarProps> = ({ onRun, onStop }) => {
  const context = useContext(Mycontext);

  if (!context) {
    throw new Error("MyContext must be used within a MyContext.Provider");
  }

  const { theme, setTheme } = context;

  return (
    <div className="bg-[#131313] flex flex-row-reverse w-full h-[8vh] basis-[1%]">
      <div className="basis-[20%]">
      <button onClick={onRun} className="w-[25px] aspect-square mt-1 "><PlayIcon className=" text-white"/></button>
      <button onClick={onStop} className="w-[25px] aspect-square ml-4 mt-1 "><StopIcon className=" text-white" /></button>
      <button onClick={() => setTheme(theme === 'vs-dark' ? 'vs-light' : 'vs-dark')}>
        {theme === 'vs-light' ? (
          <SunIcon className="w-[25px] aspect-square text-yellow-500 ml-4 mt-1 " />
        ) : (
          <MoonIcon className="w-[25px] aspect-square text-yellow-500 ml-4 mt-1" />
        )}
      </button>
      </div>
      <div className="flex flex-row basis-[80%]">
      <button className="flex flex-row basis-[20%] ml-2 mt-1">
        
        <CodeBracketIcon className="stroke-current text-white w-[25px] aspect-square"></CodeBracketIcon>
        <p className="text-sm text-white mt-1 ml-2">Code Editor</p>
      </button>
      <button className="flex flex-row basis-[20%] ml-8 mt-1">
        <CommandLineIcon className="text-white w-[25px] aspect-square"></CommandLineIcon>
        <p className="text-sm text-white mt-1 ml-2">Output</p>
        </button>
      </div>
    </div>
  );
};

export default Navbar;
