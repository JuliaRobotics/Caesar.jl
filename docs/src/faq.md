## Frequently Asked Questions

### Just-In-Time Compiling (i.e. why are first runs slow?)

Julia uses just-in-time compilation ([unless pre-compiled](https://stackoverflow.com/questions/40116045/why-is-julia-taking-a-long-time-on-the-first-call-into-my-module))
 which is slow the first time a function is called but fast from the second call onwards, since the static function is now cached and ready for use.
