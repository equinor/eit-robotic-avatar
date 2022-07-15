import { fromOffers } from "./modules/rtc.mjs";
import { pullOffers, postAnswer } from "./modules/server.mjs";
import { loadRtc } from "./modules/cameras.mjs";

async function main() {
    try {
        let offers = await pullOffers();
        console.log(offers);
        let con = await fromOffers(offers);
        let answer = await con.createAnswers();
        console.log(answer);
        await postAnswer(answer);
        let streams = con.getStreams();
        loadRtc(streams);
        console.log(streams, streams.left.getTracks(), streams.right.getTracks());
        document.getElementById("start").hidden = true;
    } catch (err) {
        console.log(err.name + ": " + err.message);
    }
}

document.getElementById("start").onclick = main;
document.getElementById("fullscreen").onclick = function () {
    document.body.requestFullscreen();
    document.getElementById("fullscreen").hidden = true;
}

