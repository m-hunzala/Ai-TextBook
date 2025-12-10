import { createOAuth2Request, encodeBasicCredentials, sendTokenRequest } from "../request.js";
const authorizationEndpoint = "https://myanimelist.net/v1/oauth2/authorize";
const tokenEndpoint = "https://myanimelist.net/v1/oauth2/token";
export class MyAnimeList {
    clientId;
    clientSecret;
    redirectURI;
    constructor(clientId, clientSecret, options) {
        this.clientId = clientId;
        this.clientSecret = clientSecret;
        this.redirectURI = options?.redirectURI ?? null;
    }
    createAuthorizationURL(state, codeVerifier) {
        const url = new URL(authorizationEndpoint);
        url.searchParams.set("response_type", "code");
        url.searchParams.set("client_id", this.clientId);
        url.searchParams.set("state", state);
        if (this.redirectURI !== null) {
            url.searchParams.set("redirect_uri", this.redirectURI);
        }
        url.searchParams.set("code_challenge_method", "plain");
        url.searchParams.set("code_challenge", codeVerifier);
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
    async refreshAccessToken(refreshToken) {
        const body = new URLSearchParams();
        body.set("grant_type", "refresh_token");
        body.set("refresh_token", refreshToken);
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
