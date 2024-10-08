import { cookies } from 'next/headers';
import axios from 'axios';

const checkCred = async () => {
  const token = cookies().get('JWTtoken')?.value;
  console.log(token)
  if (!token) {
    return { status: false, msg: 'Token not found in cookies' };
  }

  try {
    const response = await axios.get('http://localhost:8080/me', {
      headers: {
        'Authorization': `Bearer ${token}`
      }
    });
    console.log(response.data)
    //Remove this later
    if(response.data['username']==='root')
    {
      return { status: true, msg: 'Authenticated' };
    }
    else{
      return { status: false, msg: 'Not authenticated' };
    }
    //Remove this later
    if (response.data['is_authenticated']) {
      return { status: true, msg: 'Authenticated' };
    } else {
      return { status: false, msg: 'Not authenticated' };
    }
  } catch (error) {
    if (axios.isAxiosError(error)) {
      if (error.response?.status === 401) {
        return { status: false, msg: 'Authorization failed' };
      } else if (error.response?.status === 400) {
        return { status: false, msg: 'Bad Request' };
      }
    }
    return { status: false, msg: 'An unexpected error occurred' };
  }
};

export default checkCred;