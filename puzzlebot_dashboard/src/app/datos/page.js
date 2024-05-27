// pages/BackgroundPage.js

import Head from 'next/head';
import Link from 'next/link';

export default function BackgroundPage() {
  return (
    <div>
      <h1>
        DATOS
      </h1>
      <Link href="/">
              <button className="text-white bg-blue-500 px-5 py-2 rounded-md mr-4 hover:bg-slate-800">Inicio</button>
        </Link>
    </div>
  );
}
