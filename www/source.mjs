//@ts-check
import { loadCams } from "./modules/cameras.mjs";
import { fromStreams } from "./modules/rtc.mjs";
import { postOffers, pullAnswer } from "./modules/server.mjs";

async function main() {
    try {
        let cams = await loadCams();
        let con = await fromStreams(cams);
        let offers = await con.createOffers();
        console.log(offers);
        await postOffers(offers);
        let answer = await pullAnswer();
        console.log(answer);
        await con.setAnswers(answer);
    } catch(e) {
        console.log(e.name + ": " + e.message);
    }
}

main().catch(console.error);