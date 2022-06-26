import { pullOffers } from "./modules/server.mjs";

async function main() {
    try {
        let offers = await pullOffers();
        console.log(offers);
    } catch (err) {
        console.log(err.name + ": " + err.message);
    }
}

await main();

