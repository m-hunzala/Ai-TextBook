class BigEndian {
    uint8(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        return data[data.length - 1];
    }
    uint16(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        if (data.byteLength > 1) {
            return (data[data.byteLength - 2] << 8) | data[data.byteLength - 1];
        }
        return data[data.byteLength - 1];
    }
    uint32(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        let result = 0;
        for (let i = 0; i < 4; i++) {
            let byte = 0;
            if (data.byteLength - i - 1 >= 0) {
                byte = data[data.byteLength - i - 1];
            }
            result |= byte << (i * 8);
        }
        return result;
    }
    uint64(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        let result = 0n;
        for (let i = 0; i < 8; i++) {
            let byte = 0n;
            if (data.byteLength - i - 1 >= 0) {
                byte = BigInt(data[data.byteLength - i - 1]);
            }
            result |= byte << BigInt(i * 8);
        }
        return result;
    }
    putUint8(target, value, offset) {
        if (target.length < 1 + offset) {
            throw new TypeError("Not enough space");
        }
        target[offset] = value;
    }
    putUint16(target, value, offset) {
        if (target.length < 2 + offset) {
            throw new TypeError("Not enough space");
        }
        target[offset] = value >> 8;
        target[offset + 1] = value & 0xff;
    }
    putUint32(target, value, offset) {
        if (target.length < 4 + offset) {
            throw new TypeError("Not enough space");
        }
        for (let i = 0; i < 4; i++) {
            target[offset + i] = (value >> ((3 - i) * 8)) & 0xff;
        }
    }
    putUint64(target, value, offset) {
        if (target.length < 8 + offset) {
            throw new TypeError("Not enough space");
        }
        for (let i = 0; i < 8; i++) {
            target[offset + i] = Number((value >> BigInt((7 - i) * 8)) & 0xffn);
        }
    }
}
class LittleEndian {
    uint8(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        return data[0];
    }
    uint16(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        return data[0] | ((data[1] ?? 0) << 8);
    }
    uint32(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        let result = 0;
        for (let i = 0; i < 4; i++) {
            result |= (data[i] ?? 0) << (i * 8);
        }
        return result;
    }
    uint64(data) {
        if (data.byteLength === 0) {
            throw new TypeError("Empty byte array");
        }
        let result = 0n;
        for (let i = 0; i < 8; i++) {
            const byte = BigInt(data[i] ?? 0);
            result |= byte << BigInt(i * 8);
        }
        return result;
    }
    putUint8(target, value, offset) {
        if (target.length < 1 + offset) {
            throw new TypeError("Not enough space");
        }
        target[offset] = value;
    }
    putUint16(target, value, offset) {
        if (target.length < 2 + offset) {
            throw new TypeError("Not enough space");
        }
        target[offset + 1] = value >> 8;
        target[offset] = value & 0xff;
    }
    putUint32(target, value, offset) {
        if (target.length < 4 + offset) {
            throw new TypeError("Not enough space");
        }
        for (let i = 0; i < 4; i++) {
            target[offset + i] = (value >> (i * 8)) & 0xff;
        }
    }
    putUint64(target, value, offset) {
        if (target.length < 8 + offset) {
            throw new TypeError("Not enough space");
        }
        for (let i = 0; i < 8; i++) {
            target[offset + i] = Number((value >> BigInt(i * 8)) & 0xffn);
        }
    }
}
export const bigEndian = new BigEndian();
export const littleEndian = new LittleEndian();
