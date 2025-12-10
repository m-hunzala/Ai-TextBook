import { createOAuth2Request, encodeBasicCredentials, sendTokenRequest } from "../request.js";
const authorizationEndpoint = "https://slack.com/openid/connect/authorize";
const tokenEndpoint = "https://slack.com/api/openid.connect.token";
export class Slack {
    clientId;
    clientSecret;
    redirectURI;
    constructor(clientId, clientSecret, redirectURI) {
        this.clientId = clientId;
        this.clientSecret = clientSecret;
        this.redirectURI = redirectURI;
    }
    createAuthorizationURL(state, scopes) {
        const url = new URL(authorizationEndpoint);
        url.searchParams.set("response_type", "code");
        url.searchParams.set("client_id", this.clientId);
        url.searchParams.set("state", state);
        url.searchParams.set("scope", scopes.join(" "));
        if (this.redirectURI !== null) {
            url.searchParams.set("redirect_uri", this.redirectURI);
        }
        return url;
    }
    async validateAuthorizationCode(code) {
        const body = new URLSearchParams();
        body.set("grant_type", "authorization_code");
        body.set("code", code);
        if (this.redirectURI !== null) {
            body.set("redirect_uri", this.redirectURI);
        }
        const request = createOAuth2Request(tokenEndpoint, body);
        const encodedCredentials = encodeBasicCredentials(this.clientId, this.clientSecret);
        request.headers.set("Authorization", `Basic ${encodedCredentials}`);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
}
