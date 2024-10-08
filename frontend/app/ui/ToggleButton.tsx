'use client'
import { SunIcon } from "@heroicons/react/20/solid"
interface TGProps
{
    onClick : ()=> void
}
const ToggleButton : React.FC<TGProps> = ({onClick})=>{
    return (
        <button onClick={onClick} className="w-[60px] h-[40px] rounded-full flex flex-row p-[3px] align-center border-2 border-gray-400">
            <div className="w-[28px] aspect-square rounded-full bg-black"></div>
            <SunIcon className="w-[20px] h-[20px] text-yellow-500 ml-4" ></SunIcon>
        </button>
    )
}
export default ToggleButton