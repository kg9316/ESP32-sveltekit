<script lang="ts">
	import type { PageData } from './$types';
	import logo from '$lib/assets/logo.png';
	import { notifications } from '$lib/components/toasts/notifications';

	interface Props {
		data: PageData;
	}

	let { data }: Props = $props();
</script>

<div class="hero bg-base-100 h-screen">
	<div class="card md:card-side bg-base-200 shadow-primary shadow-2xl">
		<figure class="bg-base-200"><img src={logo} alt="Logo" class="h-auto w-64" /></figure>
		<div class="card-body w-80">
			<h2 class="card-title text-center text-2xl">Surfeed Demo App</h2>
			<p class="py-6 text-center">
				En enkel demo for å vise frem funksjonaliteten.
			</p>
			<a
				class="btn btn-primary"
				href="/demo"
				onclick={() => notifications.success('Start Demo !', 1000)}>Start Demo</a
			>
		</div>
	</div>
</div>
