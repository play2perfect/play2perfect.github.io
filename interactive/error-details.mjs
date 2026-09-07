// Safari stacks may contain only call locations; preserve the message separately.
export function describeError(error,operation='Starting demo'){
 const message=error?.message??String(error);
 const name=error?.name??'Error';
 const stack=typeof error?.stack==='string'?error.stack:'';
 return `${operation}\n${name}: ${message}${stack&&!stack.includes(message)?'\n'+stack:''}`;
}
