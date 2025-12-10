import { createOAuth2Request, encodeBasicCredentials, sendTokenRequest, sendTokenRevocationRequest } from "../request.js";
const authorizationEndpoint = "https://appcenter.intuit.com/connect/oauth2";
const tokenEndpoint = "https://oauth.platform.intuit.com/oauth2/v1/tokens/bearer";
const tokenRevocationEndpoint = "https://developer.API.intuit.com/v2/oauth2/tokens/revoke";
export class Intuit {
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
        url.searchParams.set("redirect_uri", this.redirectURI);
        return url;
    }
    async validateAuthorizationCode(code) {
        const body = new URLSearchParams();
        body.set("grant_type", "authorization_code");
        body.set("code", code);
        body.set("redirect_uri", this.redirectURI);
        const request = createOAuth2Request(tokenEndpoint, body);
        const encodedCredentials = encodeBasicCredentials(this.clientId, this.clientSecret);
        request.headers.set("Authorization", `Basic ${encodedCredentials}`);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
    async refreshAccessToken(refreshToken) {
        const body = new URLSearchParams();
        body.set("grant_type", "refresh_token");
        body.set("refresh_token", refreshToken);
        const request = createOAuth2Request(tokenEndpoint, body);
        const encodedCredentials = encodeBasicCredentials(this.clientId, this.clientSecret);
        request.headers.set("Authorization", `Basic ${encodedCredentials}`);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
    async revokeToken(token) {
        const body = new URLSearchParams();
        body.set("token", token);
        const request = createOAuth2Request(tokenRevocationEndpoint, body);
        const encodedCredentials = encodeBasicCredentials(this.clientId, this.clientSecret);
        request.headers.set("Authorization", `Basic ${encodedCredentials}`);
        await sendTokenRevocationRequest(request);
    }
}
