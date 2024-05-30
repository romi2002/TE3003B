'use client'
import Head from 'next/head';
import Link from 'next/link';
import {Gauge} from '@mui/x-charts/Gauge';
import {dark} from "@mui/material/styles/createPalette";
import {ThemeProvider, createTheme} from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import LidarDisplay from "@/app/LidarDisplay";
import useSWR from "swr";
import axios from 'axios';
import {CircularProgress} from "@mui/material";
import {useEffect} from "react";
import {error} from "next/dist/build/output/log";

const VelocityGauge = ({name, value}) => {
    return (
        <div className={"flex flex-col justify-center items-center"}>
            <Gauge sx={
                {'.MuiGauge-valueText': {color: 'white'}}
            } width={100} height={100} value={value} startAngle={-90} endAngle={90} valueMax={20} color="white"/>
            <span>{name}</span>
        </div>
    )
}

const LoadingSpinner = () => {
    return (
        <div className={"flex flex-col justify-center items-center"}>
            <CircularProgress />
            <span className="pt-1">Loading...</span>
        </div>
    )
}

const darkTheme = createTheme({
    palette: {
        mode: 'dark',
    },
});

const fetcher = async (api, data) => {
    try {
        const response = await axios.post(api, data);
        return response.data;
    } catch (error) {
        console.error('Error', error);
        throw error;
    }
};

export default function MainPage() {
    const { data: image_data, error: image_data_error } = useSWR("http://localhost:8042/robot_state/image", fetcher, {refreshInterval: 500, dedupingInterval: 10});
    const { data: encoder_data, error: encoder_data_error } = useSWR("http://localhost:8042/robot_state/velocity", fetcher, {refreshInterval: 500, dedupingInterval: 10});
    const { data: lidar_data, error: lidar_data_error } = useSWR("http://localhost:8042/robot_state/scan", fetcher, {refreshInterval: 500, dedupingInterval: 10});
    const lidar_size = 250;

    return (
        <ThemeProvider theme={darkTheme}>
            <CssBaseline/>
            <div className="p-10 min-h-screen flex flex-col bg-gray-600">
                <Head>
                    <title>Main Page</title>
                </Head>
                <div className="flex flex-col p-8 rounded-lg size-9/12">
                    <h1 className="text-4xl font-bold">Puzzlebot Dashboard</h1>
                    <div className="mt-5 grid grid-cols-2 gap-5">
                        <div className="flex flex-col">
                            <h2 className="text-3xl font-medium">Wheel Velocities</h2>
                            <div className="pt-5 flex items-center justify-center">
                                {encoder_data_error && <LoadingSpinner/>}
                                {encoder_data &&
                                <>
                                    <VelocityGauge name={"Left"} value={encoder_data.encL}/>
                                    <VelocityGauge name={"Right"} value={encoder_data.encR}/>
                                </>}
                            </div>
                        </div>
                        <div>
                            <span className="text-3xl font-medium">Robot Image</span>
                            <div className={"pt-5 flex items-center justify-center"}>
                                {image_data_error && <LoadingSpinner />}
                                {image_data &&
                                    <img src={`data:image/jpeg;base64,${image_data.imgB64}`} alt="Robot Image"/>
                                }
                            </div>
                        </div>
                        <div>
                            <span className="text-3xl font-medium">Lidar Data</span>
                            <div className="pt-5">
                                {lidar_data_error && <LoadingSpinner />}
                                {lidar_data &&
                                <LidarDisplay
                                    distances={lidar_data.range.map((range) => (range / lidar_data.rangeMax) * (lidar_size / 2))}
                                    // distances={lidar_data.range.map((range) => (lidar_data.rangeMax))}
                                    maxDistance={lidar_size / 2 - 5}
                                    width={lidar_size}
                                    height={lidar_size}
                                />}
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </ThemeProvider>
    );
}
