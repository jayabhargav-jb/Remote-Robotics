import { NextResponse } from "next/server";
import { cookies } from "next/headers";
export async function GET() {
    const token = cookies().get('JWTtoken')
    if(!token)
    {
        NextResponse.redirect('/')
    }
    else
    {
        const res = await fetch('http://localhost:8080/bot/iot/stop',{
            method:'GET',
            headers:{
                'Authorization': `Bearer ${token.value}`
            }
        })
        console.log(res.body)
        return new NextResponse('Stopped')
    }
}