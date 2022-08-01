import React, { ChangeEvent } from "react";
import { listDevices, loadCams } from "../modules/cameras";
import { fromOffers, fromStreams } from "../modules/rtc";
import { postAnswer, postOffers, postTracking, pullAnswer, pullOffers } from "../modules/server";
import styled, { createGlobalStyle } from "styled-components";
import Viewport, { Tracking } from "../view/Viewport";

const GlobalStyle = createGlobalStyle`
    html, body, #robotic_avatar {
        margin: 0;
        height: 100%;
    }
`

const Grid = styled.main`
    height: 100%;
    display: grid;
    box-sizing: border-box;
    grid-template-columns: 1fr;
    grid-template-rows: auto 1fr;
    grid-template-areas: 
        "ui"
        "view";
    gap: 16px 16px;
    background-color: rgb(220, 220, 220);
    padding: 8px;
`

const Ui = styled.div`
    grid-area: ui;
`

const View = styled(Viewport)`
    grid-area: view;
`

interface State {
    left?: MediaStream,
    right?: MediaStream,
    started: boolean,
    leftCamId: string,
    rightCamId: string,
    devices: [string,string][]
}

const LeftCameraId = "LeftCameraId";
const RightCameraId = "RightCameraId";

export class RoboticAvatar extends React.Component<{}, State> {
    private sending = false;

    constructor(props){
        super(props);

        const left = localStorage.getItem(LeftCameraId) ?? "";
        const right = localStorage.getItem(RightCameraId) ?? "";
        
        this.state = {
            started: false,
            leftCamId: left,
            rightCamId: right,
            devices: []
        }

        listDevices().then(devices => this.setState({devices: devices}))
    }

    render(): React.ReactNode {
        const devices = this.state.devices.map(device => <li>{device[0]}: {device[1]}</li>)

        return <Grid>
            <GlobalStyle/>
            <Ui>
                <h1>Robotic Avatar Demo</h1>
                <p>
                    Left Camera ID: <input size={64} value={this.state.leftCamId} onChange={this.handleLeftCam} /><br/>
                    Right Camera ID <input size={64} value={this.state.rightCamId} onChange={this.handleRightCam} />
                    <ul>{devices}</ul>
                </p>
                <p>
                    <button disabled={this.state.started} onClick={this.handleSource}>Start as source</button>
                    <button disabled={this.state.started} onClick={this.handleSourceNoView}>Start as source NO VIEWPORT</button>
                    <button disabled={this.state.started} onClick={this.handleReceiver}>Start as receiver</button> 
                </p>
            </Ui>
            <View left={this.state.left} right={this.state.right} onTrack={this.handleTracking}/> 
        </Grid>
    }

    handleLeftCam = (event: ChangeEvent<HTMLInputElement>) => {
        const value = event.target.value;
        localStorage.setItem(LeftCameraId, value);
        this.setState({leftCamId: value});    
    }

    handleRightCam = (event: ChangeEvent<HTMLInputElement>) => {
        const value = event.target.value;
        localStorage.setItem(RightCameraId, value);
        this.setState({rightCamId: value});    
    }

    handleSource = async () => {
        try{
            this.setState({started: true});
            let cams = await loadCams(this.state.leftCamId, this.state.rightCamId);
            this.setState(cams);
            this.setState({devices: await listDevices()});
            let con = await fromStreams(cams);
            let offers = await con.createOffers();
            console.log(offers);
            await postOffers(offers);
            let answer = await pullAnswer();
            console.log(answer);
            await con.setAnswers(answer);
        } catch (err) {
            console.error(err)
        }
    }

    handleSourceNoView = async () => {
        try{
            this.setState({started: true});
            let cams = await loadCams(this.state.leftCamId, this.state.rightCamId);
            // this.setState(cams);
            this.setState({devices: await listDevices()});
            let con = await fromStreams(cams);
            let offers = await con.createOffers();
            console.log(offers);
            await postOffers(offers);
            let answer = await pullAnswer();
            console.log(answer);
            await con.setAnswers(answer);
        } catch (err) {
            console.error(err)
        }
    }

    handleReceiver = async () => {
        try {
            this.setState({started: true});
            let offers = await pullOffers();
            console.log(offers);
            let con = await fromOffers(offers);
            let answer = await con.createAnswers();
            console.log(answer);
            await postAnswer(answer);
            let streams = con.getStreams();
            this.setState(streams);
        } catch (err) {
            console.log(err);
        }
    } 

    handleTracking = async (track: Tracking) => {
        try {
            if(this.sending) return;
            this.sending = true;
            await postTracking(track);
            this.sending = false;
        } catch (err) {
            console.log(err);
        }
    } 
}