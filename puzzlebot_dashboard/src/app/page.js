'use client'
import useSWR from "swr";
import { useState, useEffect } from 'react'
import axios from 'axios';

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
  const { data: image_data, error, isLoading } = useSWR("http://localhost:8042/robot_state/image", fetcher, {refreshInterval: 10, dedupingInterval: 5});

  console.log(image_data)
  return (
    <main className="flex min-h-screen flex-col items-center justify-between p-24">
      <div className="z-10 max-w-5xl w-full items-center justify-between font-mono text-sm lg:flex">
        <h1 className="text-4xl">Puzzlebot Dashboard</h1>
        {isLoading && <p>Loading...</p>}
        {error && <p>Error: {error}</p>}
        {image_data && (
          <img src={`data:image/jpeg;base64,${image_data.imgB64}`} alt="Robot Image" />
        )}
      </div>
    </main>
  );
}
