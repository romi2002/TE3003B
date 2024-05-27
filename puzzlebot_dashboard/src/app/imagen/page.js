'use client'
import useSWR from "swr";
import { useState, useEffect } from 'react'
import axios from 'axios';
import Link from 'next/link';

const fetcher = async (api, data) => {
  try {
    const response = await axios.post(api, data);
    console.log(response)
    return response.data;
  } catch (error) {
    console.error('Error', error);
  }
};

export default function Home() {
  const { data: image_data, error, isLoading } = useSWR("http://localhost:8042/robot_state/image", fetcher, { refreshInterval: 10, dedupingInterval: 5 });

  console.log(image_data)
  return (
    <main className="flex min-h-screen flex-col items-center justify-center p-24">
      <div className="z-10 max-w-5xl w-full flex flex-col items-center font-mono text-sm lg:flex">
        <h1 className="text-4xl mb-8">Change to the image in use</h1>   
        {isLoading && <p>Loading...</p>} 
        {error && <p>Error: {error}</p>}
        {image_data && (
          <img src={`data:image/jpeg;base64,${image_data.imgB64}`} alt="Robot Image" className="w-full max-w-screen-xl" />   /* Adjust max-width here */
        )}
        <Link href="/">
          <button className="text-white bg-gray-200 bg-opacity-80 px-6 py-3 rounded-md mr-4 mt-20 hover:bg-slate-2000">Inicio</button>
        </Link>
      </div>
    </main>
  );
}
