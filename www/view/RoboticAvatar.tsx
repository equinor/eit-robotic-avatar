import React from "react";
import { loadCams } from "../modules/cameras";
import { fromOffers, fromStreams } from "../modules/rtc";
import { postAnswer, postOffers, pullAnswer, pullOffers } from "../modules/server";
import styled, { createGlobalStyle } from "styled-components";
import Viewport from "../view/Viewport";

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
}

export class RoboticAvatar extends React.Component<{}, State> {
    constructor(props){
        super(props);       
        this.state = {
            started: false,
        }
    }

    render(): React.ReactNode {
        return <Grid>
            <GlobalStyle/>
            <Ui>
                <h1>Robotic Avatar Level 1 demo</h1>
                <p>
                    <button disabled={this.state.started} onClick={this.handleSource}>Start as source</button>
                    <button disabled={this.state.started} onClick={this.handleSourceNoView}>Start as source NO VIEWPORT</button>
                    <button disabled={this.state.started} onClick={this.handleReceiver}>Start as receiver</button> 
                </p>
            </Ui>
            <View left={this.state.left} right={this.state.right} onTrack={console.log}/> 
        </Grid>
    }

    handleSource = async () => {
        try{
            this.setState({started: true});
            let cams = await loadCams();
            this.setState(cams);
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
            let cams = await loadCams();
            // this.setState(cams);
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
}