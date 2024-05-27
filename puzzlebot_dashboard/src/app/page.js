// pages/BackgroundPage.js

import Head from 'next/head';
import Link from 'next/link';

export default function BackgroundPage() {
  return (
    <div className="bg-contain bg-center min-h-screen flex flex-col items-center justify-center relative" style={{backgroundImage: "url('https://manchester-robotics.com/wp-content/uploads/puzzle-3-edited-1024x1024.jpg')"}}>
      <Head>
        <title>Background Page</title>
      </Head>
      <div className="absolute inset-0 flex flex-col items-center justify-center z-10 bg-black bg-opacity-30">
        <div className="p-8 rounded-lg text-white">
          <h1 className="text-4xl font-bold">Puzzlebot Dashboard</h1>
          <p className="text-lg mt-2">This is the home page, view the options</p>
          <div className="mt-8 space-x-10">
            <Link href="/imagen">
              <button className="text-white bg-blue-500 px-5 py-2 rounded-md mr-4 hover:bg-slate-800">Imagen</button>
            </Link>
            <Link href="/datos">
              <button className="text-white bg-blue-500 px-5 py-2 rounded-md hover:bg-slate-800">Datos</button>
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}
