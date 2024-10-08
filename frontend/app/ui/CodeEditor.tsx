'use client';
import { useEffect, useState,useRef, createContext} from "react";
import Editor ,{useMonaco} from '@monaco-editor/react';
import * as monaco from 'monaco-editor'
import Navbar from "./Navbar";

interface MyContextType {
  theme: 'vs-light' | 'vs-dark';
  setTheme: React.Dispatch<React.SetStateAction<"vs-light" | "vs-dark">>;
}

export const Mycontext = createContext<MyContextType | undefined>(undefined);

export default function CodeEditor() {
  const [theme, setTheme] = useState<'vs-light' | 'vs-dark'>('vs-dark');
  const monaco = useMonaco();
 
  const editorRef = useRef<monaco.editor.IStandaloneCodeEditor | null>(null);
  //const editorInstanceRef = useRef<monaco.editor.IStandaloneCodeEditor | null>(null);
  function handleEditorMount(editor:monaco.editor.IStandaloneCodeEditor)
  {
    editorRef.current =editor;
  }
    
  useEffect(() => {
    if (monaco) {
      monaco.editor.setTheme(theme)
    }
  }, [monaco,theme]);

  /*useEffect(() => {
    if (editorRef.current) {
      // Create the editor instance on initial mount
      const editor = monaco.editor.create(editorRef.current, {
        value: `print("Hello, world!")`,
        language: 'python',
        theme: theme,
      });

      editorInstanceRef.current = editor;

      // Cleanup on unmount
      return () => editor.dispose();
    }
  }, []);
*/
  // Effect to update the theme whenever it changes

  useEffect(() => {

  }, [theme]);
  const handleRun = async () => {
    
    if (editorRef.current) {
      console.log('Running Code:', monaco);
      const code = editorRef.current.getValue();
      console.log(code);
      const j = { code: code };
      try {
        const res = await fetch('/api/file', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(j)
        });
        //const b = await res.blob()


        if (!res.ok) {
          throw new Error(`HTTP error! status: ${res.status}`);
        }
        //const data = await res.json();
        console.log('Response Cameeee:');
      } catch (error) {
        console.error('Error:', error);
      }
    }

  };

  const handleStop = async () => {
    console.log('Stopping execution...');
    const res = await fetch('/api/stop',{method:'GET'})
    console.log(res.body)
  };
  //<div ref={editorRef}  className="w-full basis-[99%]"></div>
  return (
    <Mycontext.Provider value={{ theme, setTheme }}>
      <div className="h-screen flex flex-row-reverse">
        <div className="basis-1/2 flex flex-col border-2">
          <Navbar onRun={handleRun} onStop={handleStop} />
          <Editor defaultLanguage="python" defaultValue="#Rero" theme={theme} onMount={handleEditorMount} ></Editor>
        </div>
        <div className="basis-1/2"></div>
      </div>
    </Mycontext.Provider>
  );
}
