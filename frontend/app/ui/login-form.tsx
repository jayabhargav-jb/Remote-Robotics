'use client';
import axios from 'axios';

import { lusitana } from '@/app/ui/fonts';
import { ArrowRightIcon, AtSymbolIcon ,KeyIcon,ClockIcon} from '@heroicons/react/20/solid';
import { Button } from './button';
import React, { useState } from 'react';
import { useRouter } from 'next/navigation';
//import Cookies from 'js-cookie'
import { setCookie } from 'cookies-next';
export default function LoginForm() {
  const [username,setUsername]= useState('');
  const [password,setPassword] =useState('');
  const [iserr,setError]= useState(false)
  const [emsg,setEmsg]= useState('')
  const router = useRouter();
  const handleSubmit = async (e:React.FormEvent)=>{
    e.preventDefault();
    try{
      const res = await axios.post('/api/login',{username,password})
        console.log('Login successful:', res.data);
      if(res.status===200)
      {
        const token:string = res.data.access_token
        console.log(token)
        setCookie('JWTtoken', token, { maxAge: 60 * 60 * 24, secure: false,sameSite:'lax'});
        router.push('/dashboard')
      }else{
        console.log(res.data)
        setError(true);
        setEmsg(res.data['detail']['message']+' '+res.data['detail']['timeslot_start']+' '+res.data['detail']['timeslot_end'])
      }
    }catch (error) {
      // Handle error response
      console.error(error);
      setError(true);
      if (axios.isAxiosError(error) && error.response) {
        switch (error.response.status) {
          case 400:
            setEmsg('Bad Request. Please check your input.');
            break;
          case 401:
            setEmsg('Unauthorized. Please check your credentials.');
            break;
          case 403:
            setEmsg('Forbidden. You do not have permission to access this resource.');
            break;
          case 409:
            setEmsg('Please wait for your time ');
            break;
          case 500:
            setEmsg('Internal Server Error. Please try again later.');
            break;
          default:
            setEmsg('An unexpected error occurred.');
            break;
        }
    }
  }
  }
  return (
    <form  onSubmit ={handleSubmit} className="space-y-3">
      <div className="flex-1 rounded-lg bg-gray-50 px-6 pb-4 pt-8">
        <h1 className={`${lusitana.className} mb-3 text-2xl`}>
          Please log in to continue.
        </h1>
        <div className="w-full">
          <div>
            <label
              className="mb-3 mt-5 block text-xs font-medium text-gray-900"
              htmlFor="username"
            >
              Username
            </label>
            <div className="relative">
              <AtSymbolIcon className="absolute left-3 top-1/2 transform -translate-y-1/2 h-5 w-5 text-gray-400" />
              <input
                className="peer block w-full rounded-md border border-gray-200 py-[9px] pl-10 text-sm outline-2 placeholder:text-gray-500"
                id="username"
                type="username"
                name="username"
                placeholder="rero123"
                onChange={(e)=>{setUsername(e.target.value)}}
              />
            </div>
            <div className="mt-4">
            <label
              className="mb-3 mt-5 block text-xs font-medium text-gray-900"
              htmlFor="password"
            >
              Password
            </label>
            <div className="relative">
              <input
                className="peer block w-full rounded-md border border-gray-200 py-[9px] pl-10 text-sm outline-2 placeholder:text-gray-500"
                id="password"
                type="password"
                name="password"
                placeholder="Enter password"
                required
                minLength={6}
                onChange={(e)=>{setPassword(e.target.value)}}
              />
              <KeyIcon className="pointer-events-none absolute left-3 top-1/2 h-[18px] w-[18px] -translate-y-1/2 text-gray-500 peer-focus:text-gray-900" />
            </div>
          </div>
          </div>
          <Button type="submit" className="mt-4 flex items-center justify-center" >
            Login
            <ArrowRightIcon className="ml-2 h-5 w-5" />
          </Button>
          <Button  className="mt-4 flex items-center justify-center" >
            Check Timeslot
            <ClockIcon className="ml-2 h-5 w-5" />
          </Button>
          {iserr && <p className='mt-4 text-red-500'>{emsg}</p>}
          <div
          className="flex h-8 items-end space-x-1"
          aria-live="polite"
          aria-atomic="true"
        >
        </div>
        </div>
      </div>
    </form>
  );
}