import { NextRequest,NextResponse} from "next/server";
import { cookies } from "next/headers";
interface BodyParams
{
    code : string
}
export  async function POST(request:NextRequest) {
    const token = cookies().get('JWTtoken')
    if(!token)
    {
        NextResponse.redirect('/')
    }
    else
    {
        const body:BodyParams = await request.json();
        console.log(body.code)
        //const fileBuffer = Buffer.from(body.code,'utf8')
        //console.log(fileBuffer)
        const blob = new Blob([body.code],{type:'text/x-python'})
        const formData = new FormData();
        formData.append('file',blob)
        try
        {
            const res = await fetch('http://localhost:8080/bot/iot/code',{
                method:'POST',
                headers:{
                    'accept': 'application/json',
                    'Authorization' : `Bearer ${token.value}`
                },
                body : formData
            })
            const reader = res.body?.getReader();
            let resBody = '';
            if (reader) {
                const decoder = new TextDecoder();
                let done = false;
                while (!done) {
                    const { value, done: doneReading } = await reader.read();
                    done = doneReading;
                    if (value) {
                        resBody += decoder.decode(value, { stream: !done });
                    }
                }
            }
            console.log(resBody);
            if(!res.ok)
            {
                return new NextResponse("File not sent")
            }
            return new NextResponse("File Sent")
        }
        catch(error)
        {
            console.log("File not sent",error)
            return new NextResponse("File not sent")
        }
    }
    
    

    
}