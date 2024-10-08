

//import { useEffect, useState } from 'react';
import { redirect } from 'next/navigation';
//import axios from 'axios';
import checkCred from '../functions/functions';
import CodeEditor from '../ui/CodeEditor';
export default async function Page() {
  const res = await checkCred()
  console.log(res)
  if(res['status']===false)
  {
    redirect('/')
  }
  /*
  const [loading, setLoading] = useState(true);
  const router = useRouter();

  useEffect(() => {
    const checkAuth = async () => {
      try {
        const response = await axios.get('/api/check');
        if (response.status === 200) {

          setLoading(false);
        } else {
          router.push('/login');
        }
      } catch (error) {
        console.error('Authentication check failed:', error);
        router.push('/login');
      }
    };

    checkAuth();
  }, [router]);

  if (loading) {
    return <div>Loading...</div>;
  }
*/
  return (
    <CodeEditor></CodeEditor>
    //<ToggleButton onClick={handleClick}></ToggleButton>
  )
}