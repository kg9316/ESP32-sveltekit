import type { PageLoad } from './$types';

export const load = (async ({ fetch }) => {
	return {
		title: 'Surfeeder Demo',
	};
}) satisfies PageLoad;
