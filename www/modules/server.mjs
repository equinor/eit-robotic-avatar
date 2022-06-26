//@ts-check

export async function postOffers(offers) {
    await fetch("./post_offer", {
        method: 'POST',
        headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json'
        },
        body: JSON.stringify(offers)
    });
}

// pull and server until we get something looks like an offer.
export async function pullOffers() {
    while (true) {
        let req = await fetch("./get_offer");
        let offer = await req.json();
        if (offer != undefined && offer.left != undefined) {
            return offer;
        }
        // Wait 5 sec
        await new Promise(resolve => setTimeout(resolve, 5000));
    }
}