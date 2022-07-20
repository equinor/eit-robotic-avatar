import React from "react";
import { loadCams } from "./modules/cameras";
import { fromStreams } from "./modules/rtc";
import { postOffers, pullAnswer } from "./modules/server";
import { createRoot } from 'react-dom/client';
import styled, { createGlobalStyle } from "styled-components";

const GlobalStyle = createGlobalStyle`
    html, body, #robotic_avatar {
        margin: 0;
    }
`

const LeftVideo = styled.video`
    position: absolute;
    transform: rotate(270deg);
    transform-origin: top left;
    width: 100vh;
    height: 50vw;
    margin-top: 100vh;
    object-fit: cover;
`

const RightVideo = styled.video`
    position: absolute;
    transform: rotate(90deg);
    transform-origin: bottom left;
    width: 100vh;
    height: 50vw;
    margin-top: -50vw;
    margin-left: 50vw;
    object-fit: cover;
`

class RoboticAvatar extends React.Component {
    render(): React.ReactNode {
        return <>
            <GlobalStyle/>
            <LeftVideo autoPlay id="lefttag"/>
            <RightVideo autoPlay id="righttag"/>
        </>
    }

    componentDidMount() {
        const inner = async () => {
            let cams = await loadCams();
            let con = await fromStreams(cams);
            let offers = await con.createOffers();
            console.log(offers);
            await postOffers(offers);
            let answer = await pullAnswer();
            console.log(answer);
            await con.setAnswers(answer);
        }
        
        inner().catch(console.error);
    }
}

const react_root = createRoot(document.getElementById("robotic_avatar")!);
react_root.render(<RoboticAvatar/>);
