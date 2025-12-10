import { Store, StoreValue } from 'nanostores';
import { Accessor } from 'solid-js';

/**
 * Subscribes to store changes and gets store’s value.
 *
 * @param store Store instance.
 * @returns Store value.
 */
declare function useStore<SomeStore extends Store, Value extends StoreValue<SomeStore>>(store: SomeStore): Accessor<Value>;

export { useStore };
