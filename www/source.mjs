//@ts-check
import { loadCams } from "./modules/cameras.mjs";
import { fromStreams } from "./modules/rtc.mjs";

async function main() {
    try {
        let cams = await loadCams();
        let con = await fromStreams(cams);
        console.log(await con.createOffers());
    } catch(e) {
        console.log(e.name + ": " + e.message);
    }
}

await main();