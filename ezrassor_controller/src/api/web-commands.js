export default class HTTP {

    static doPost(host, message, extension) {

        console.log("MESSAGE: ", message, " IP ADDRESS => ", host, "Extension: ", extension);
        // Adding http:// to the IP address
        let ip_adrs = "http://" + host + extension;
        console.log(ip_adrs);

        return fetch(
            ip_adrs,
            {
                method: 'POST',
                headers: {
                    Accept: 'application/json',
                    'Content-Type': 'application/json',
                },
                body: message
            }
        )
            .catch((error) => {
                console.log("web-command displaying post error: ", error);
            });
    }

    static doGet(host, extension) {
        console.log("IP ADDRESS => ", host, " Extension: ", extension);
        // Adding http:// to the IP address
        let ip_adrs = "http://" + host + extension;
        console.log(ip_adrs);

        return fetch(ip_adrs, {
            method: 'GET',  // Use GET method instead of POST
            headers: {
                Accept: 'application/json',
            },
        })
        .then(response => response.json())  // Parse the response as JSON
        .then(data => {
            console.log("Received data:", data);
            return data;
        })
        .catch((error) => {
            console.log("Error while fetching command status: ", error);
        });
    }


}