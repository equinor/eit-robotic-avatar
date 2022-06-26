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
        loadRtc(con.getStreams());
    } catch (err) {
        console.log(err.name + ": " + err.message);
    }
}

await main();

