export function xor(target, mask) {
    if (target.byteLength !== mask.byteLength) {
        throw new TypeError("Byte length do not match");
    }
    for (let i = 0; i < target.byteLength; i++) {
        target[i] ^= mask[i];
    }
}
export function or(target, mask) {
    if (target.byteLength !== mask.byteLength) {
        throw new TypeError("Byte length do not match");
    }
    for (let i = 0; i < target.byteLength; i++) {
        target[i] |= mask[i];
    }
}
export function and(target, mask) {
    if (target.byteLength !== mask.byteLength) {
        throw new TypeError("Byte length do not match");
    }
    for (let i = 0; i < target.byteLength; i++) {
        target[i] &= mask[i];
    }
}
export function not(target) {
    for (let i = 0; i < target.byteLength; i++) {
        target[i] = ~target[i];
    }
}
export function rotl32(x, n) {
    return ((x << n) | (x >>> (32 - n))) >>> 0;
}
export function rotr32(x, n) {
    return ((x << (32 - n)) | (x >>> n)) >>> 0;
}
export function rotr64(x, n) {
    return ((x << BigInt(64 - n)) | (x >> BigInt(n))) & 0xffffffffffffffffn;
}
export function rotl64(x, n) {
    return ((x << BigInt(n)) | (x >> BigInt(64 - n))) & 0xffffffffffffffffn;
}
